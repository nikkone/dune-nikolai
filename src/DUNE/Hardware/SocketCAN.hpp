#ifndef DUNE_HARDWARE_SOCKETCAN_HPP_INCLUDED_
#define DUNE_HARDWARE_SOCKETCAN_HPP_INCLUDED_
/* TODO/missing functionality:
* Verify that canbus is set up and opened(Give error if its not in ifconfig)
* Implement flush functions

*/
// DUNE headers.
#include <DUNE/Config.hpp>
#include <DUNE/IO/Handle.hpp>

#if defined(DUNE_OS_LINUX)
#include <linux/can.h>
#include <net/if.h>
#else
      throw Error("Unimplemented feature", "DUNE::Hardware::SocketCAN");
#endif


namespace DUNE
{
  namespace Hardware
  {
    // Export DLL Symbol.
    class DUNE_DLL_SYM SocketCAN;

    //! The SocketCAN class encapsulates CAN access.
    class SocketCAN: public IO::Handle
    {
    public:
      class Error: public std::runtime_error
      {
      public:
        Error(std::string op, std::string msg):
          std::runtime_error("Socket CAN error (" + op + "): " + msg)
        { }
      };
#if defined(DUNE_OS_LINUX)
      enum can_frame_t {
        CAN_BASIC_SFF = 0,
        CAN_BASIC_EFF = 1,
        CAN_FD = 2
      };
      //! SocketCAN constructor.
      SocketCAN(const std::string& can_dev, can_frame_t frame_type);
      
      //! Socket CAN destructor.
      ~SocketCAN(void);
      
      void setTXID(uint32_t id);
      uint32_t getRXID();
      size_t readHexString(char* bfr, size_t length);
      //size_t writeHexString(const char* cstr);
    private:
      //! CAN connection variables.
      struct sockaddr_can m_addr;
      struct ifreq m_ifr;
      int m_can_socket;
      can_frame_t can_frame_type;
      uint32_t cantxid = 0;
      uint32_t canrxid = 0;
      
      IO::NativeHandle
      doGetNative(void) const
      {
        return m_can_socket; // Makes Poll::poll work
      }

      size_t
      doWrite(const uint8_t* bfr, size_t size);

      size_t
      doRead(uint8_t* bfr, size_t size);

      //! Flush input buffer, discarding all of it's contents.
      void
      doFlushInput(void);

      //! Flush output buffer, aborting output.
      void
      doFlushOutput(void);

      //! Flush both input and output buffers.
      void
      doFlush(void);
#else
      throw Error("Unimplemented feature", "DUNE::Hardware::SocketCAN");
#endif
    };
  }
}


#endif
