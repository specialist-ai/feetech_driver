// One tty, one bus, however many Bus objects name it.
//
// Two grippers may share one adapter, so two Gripper objects open the SAME
// device. Bus resolves every open through a process-wide registry keyed on the
// device name, so the two share one fd and one transaction mutex.
//
// The lifetime is the part worth pinning: the fd belongs to the LAST holder, not
// the first. A close() that closed the fd outright would leave the other holder
// reading a closed descriptor -- which surfaces as a gripper that goes silent
// when the other one is torn down, and only on the shared wiring. These tests
// run against a pty, so they need no serial hardware.

#include <gtest/gtest.h>

#include <csignal>
#include <pty.h>
#include <sys/wait.h>
#include <unistd.h>

#include <string>

#include "feetech/bus.hpp"

namespace
{

// A pty pair stands in for the adapter: the slave side is a real tty that takes
// termios settings, and nothing ever answers a request on it.
class PtyDevice
{
public:
  PtyDevice()
  {
    char name[256] = {};
    if (::openpty(&master_, &slave_, name, nullptr, nullptr) == 0) {
      name_ = name;
    }
  }

  ~PtyDevice()
  {
    if (master_ >= 0) {::close(master_);}
    if (slave_ >= 0) {::close(slave_);}
  }

  const std::string & name() const {return name_;}

private:
  int master_{-1};
  int slave_{-1};
  std::string name_;
};

}  // namespace

using feetech::Bus;

TEST(BusSharing, TwoBusesOpenOneDevice)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  Bus left;
  Bus right;
  ASSERT_TRUE(left.open(device.name(), 1000000)) << left.last_error();
  ASSERT_TRUE(right.open(device.name(), 1000000)) << right.last_error();
  EXPECT_TRUE(left.is_open());
  EXPECT_TRUE(right.is_open());
}

TEST(BusSharing, ClosingOneLeavesTheOtherUsable)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  Bus left;
  Bus right;
  ASSERT_TRUE(left.open(device.name(), 1000000)) << left.last_error();
  ASSERT_TRUE(right.open(device.name(), 1000000)) << right.last_error();

  left.close();

  // Nothing answers a pty, so the ping fails either way -- what separates a
  // shared fd that survived from one that was closed underneath is WHY. A live
  // fd times out waiting for a reply; a closed one reports "bus not open" (or
  // fails in write()).
  right.set_timeout_ms(20);
  EXPECT_FALSE(right.ping(18));
  EXPECT_NE(right.last_error().find("timeout"), std::string::npos)
    << "expected a timeout on a live fd, got: " << right.last_error();
  EXPECT_TRUE(right.is_open());
}

TEST(BusSharing, ReopenAfterEveryHolderClosed)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  {
    Bus first;
    ASSERT_TRUE(first.open(device.name(), 1000000)) << first.last_error();
  }
  // The registry holds a weak reference only, so the expired entry must not
  // block a later open of the same device.
  Bus second;
  ASSERT_TRUE(second.open(device.name(), 1000000)) << second.last_error();
  EXPECT_TRUE(second.is_open());
}

TEST(BusSharing, SecondOpenAtAnotherBaudIsRefused)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  Bus left;
  Bus right;
  ASSERT_TRUE(left.open(device.name(), 1000000)) << left.last_error();

  // Every servo on one wire runs at one rate, so this is a config error worth a
  // refusal rather than a silently reconfigured tty under the first component.
  EXPECT_FALSE(right.open(device.name(), 115200));
  EXPECT_NE(right.last_error().find("already open"), std::string::npos)
    << right.last_error();
  EXPECT_TRUE(left.is_open());
}

// --- exclusion across processes -------------------------------------------
//
// The registry above is a process-wide static, so it can do nothing about a
// second PROGRAM on the same wire. That case corrupts identically and silently:
// reads steal each other's bytes, checksums fail, and the driver faults and
// releases torque, which reads as a broken servo. open() therefore also takes
// an advisory flock.

namespace
{

/// A second process that tries to open `device` when told to.
///
/// It has to be forked BEFORE this process opens the device. A child forked
/// afterwards inherits both the file descriptor and the process-wide registry,
/// so its Bus::open would find the parent's Port and succeed without touching
/// the wire -- it would not be a second program at all.
class Contender
{
public:
  explicit Contender(std::string device)
  : device_(std::move(device))
  {
    EXPECT_EQ(pipe(to_child_), 0);
    EXPECT_EQ(pipe(to_parent_), 0);
    pid_ = fork();
    EXPECT_GE(pid_, 0);
    if (pid_ == 0) {
      ::close(to_child_[1]);
      ::close(to_parent_[0]);
      char go = 0;
      const bool ready = ::read(to_child_[0], &go, 1) == 1;
      Bus bus;
      const char verdict = (ready && bus.open(device_, 1000000)) ? 'y' : 'n';
      [[maybe_unused]] ssize_t written = ::write(to_parent_[1], &verdict, 1);
      // Hold the device until the parent has looked, then leave.
      [[maybe_unused]] ssize_t waited = ::read(to_child_[0], &go, 1);
      _exit(0);
    }
    ::close(to_child_[0]);
    ::close(to_parent_[1]);
  }

  ~Contender()
  {
    if (pid_ > 0) {
      const char go = 1;
      [[maybe_unused]] ssize_t released = ::write(to_child_[1], &go, 1);
      int status = 0;
      ::waitpid(pid_, &status, 0);
    }
    ::close(to_child_[1]);
    ::close(to_parent_[0]);
  }

  pid_t pid() const {return pid_;}

  /// Let the child try, and report whether it got the device.
  bool opened_it()
  {
    const char go = 1;
    EXPECT_EQ(::write(to_child_[1], &go, 1), 1);
    char verdict = '?';
    EXPECT_EQ(::read(to_parent_[0], &verdict, 1), 1);
    return verdict == 'y';
  }

private:
  std::string device_;
  pid_t pid_{-1};
  int to_child_[2]{-1, -1};
  int to_parent_[2]{-1, -1};
};

}  // namespace

TEST(BusExclusion, ASecondProcessCannotOpenTheSameDevice)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  Contender other(device.name());
  Bus held;
  ASSERT_TRUE(held.open(device.name(), 1000000)) << held.last_error();

  EXPECT_FALSE(other.opened_it()) << "a second process opened a device this one holds";
}

TEST(BusExclusion, TheDeviceIsFreeAgainOnceTheHolderCloses)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  Contender other(device.name());
  {
    Bus held;
    ASSERT_TRUE(held.open(device.name(), 1000000)) << held.last_error();
  }
  EXPECT_TRUE(other.opened_it());
}

// The refusal has to say who is holding the bus. "Busy" alone sends an operator
// looking at the wiring.
TEST(BusExclusion, TheRefusalNamesTheHolder)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  Contender other(device.name());
  ASSERT_TRUE(other.opened_it());

  Bus refused;
  EXPECT_FALSE(refused.open(device.name(), 1000000));
  EXPECT_NE(refused.last_error().find("pid " + std::to_string(other.pid())), std::string::npos)
    << refused.last_error();
}

// The lock lives on the open file description, so the kernel drops it when the
// process dies. A lock FILE would go stale here and need a liveness check.
TEST(BusExclusion, ADeadHolderLeavesNoStaleLock)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  int to_parent[2];
  ASSERT_EQ(pipe(to_parent), 0);
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(to_parent[0]);
    Bus bus;
    const char verdict = bus.open(device.name(), 1000000) ? 'y' : 'n';
    [[maybe_unused]] ssize_t written = ::write(to_parent[1], &verdict, 1);
    ::raise(SIGKILL);       // no destructor runs, no close() is reached
    _exit(0);
  }
  ::close(to_parent[1]);
  char verdict = '?';
  ASSERT_EQ(::read(to_parent[0], &verdict, 1), 1);
  ASSERT_EQ(verdict, 'y');
  int status = 0;
  ::waitpid(child, &status, 0);
  ::close(to_parent[0]);

  Bus after;
  EXPECT_TRUE(after.open(device.name(), 1000000)) << after.last_error();
}

// /dev/ttyUSB0 and a /dev/serial/by-id symlink to it are one device. The
// registry keys on the resolved path, so two names share one fd and one
// transaction mutex rather than opening the wire twice inside one process.
TEST(BusSharing, TwoNamesForOneDeviceShareIt)
{
  PtyDevice device;
  ASSERT_FALSE(device.name().empty()) << "openpty failed";

  const std::string alias = "/tmp/feetech_bus_alias_" + std::to_string(::getpid());
  ::unlink(alias.c_str());
  ASSERT_EQ(::symlink(device.name().c_str(), alias.c_str()), 0);

  Bus direct;
  Bus through_alias;
  ASSERT_TRUE(direct.open(device.name(), 1000000)) << direct.last_error();
  ASSERT_TRUE(through_alias.open(alias, 1000000)) << through_alias.last_error();
  EXPECT_TRUE(direct.is_open());
  EXPECT_TRUE(through_alias.is_open());

  ::unlink(alias.c_str());
}
