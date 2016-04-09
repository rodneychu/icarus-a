#ifndef _ant_h_
#define _ant_h_

#include "Usb.h"

// To enable serial debugging see "settings.h"
//#define EXTRADEBUG // Uncomment to get even more debugging data

#define ANT_CTRL_PIPE 0
#define ANT_DATA_IN_PIPE 1
#define ANT_DATA_OUT_PIPE 2
#define ANT_MAX_ENDPOINTS 3

typedef void (*DataInCallback) (uint8_t* buf, uint16_t bytes);

/**
 * The Ant Dongle class will take care of all the USB communication
 * and then pass the data to the BluetoothService classes.
 */
class Ant : public USBDeviceConfig, public UsbConfigXtracter {
public:
  /**
   * Constructor for the Ant class.
   * @param  p   Pointer to USB class instance.
   */
  Ant(USB *p);

  /**
   * Initialize the Ant dongle.
   * @param  parent   Hub number.
   * @param  port     Port number on the hub.
   * @param  lowspeed Speed of the device.
   * @return          0 on success.
   */
  uint8_t Init(uint8_t parent, uint8_t port, bool lowspeed);
  /** @name USBDeviceConfig implementation */
  /**
   * Address assignment and basic initialization is done here.
   * @param  parent   Hub number.
   * @param  port     Port number on the hub.
   * @param  lowspeed Speed of the device.
   * @return          0 on success.
   */
  uint8_t ConfigureDevice(uint8_t parent, uint8_t port, bool lowspeed);
  /**
   * Release the USB device.
   * @return 0 on success.
   */
  uint8_t Release();
  /**
   * Poll the USB Input endpoints and run the state machines.
   * @return 0 on success.
   */
  uint8_t Poll();

  /**
   * Get the device address.
   * @return The device address.
   */
  virtual uint8_t GetAddress() {
          return device_address_;
  };
  
  /**
   * Used to check if the dongle has been initialized.
   * @return True if it's ready.
   */
  virtual bool is_ready() {
      return poll_enable_;
  };
  
  /**
   * Used by the USB core to check what this driver support.
   * @param  klass The device's USB class.
   * @return       Returns true if the device's USB class matches this driver.
   */
  virtual bool DEVCLASSOK(uint8_t klass) {
          return (klass == 0);
  };

  /**
   * Used by the USB core to check what this driver support.
   * @param  vid The device's VID.
   * @param  pid The device's PID.
   * @return     Returns true if the device's VID and PID matches this driver.
   */
  virtual bool VIDPIDOK(uint16_t vid, uint16_t pid) {
          return vid == 0x0fcf && pid == 0x1009;
  };
  /**@}*/

  /** @name UsbConfigXtracter implementation */
  /**
   * UsbConfigXtracter implementation, used to extract endpoint information.
   * @param conf  Configuration value.
   * @param iface Interface number.
   * @param alt   Alternate setting.
   * @param proto Interface Protocol.
   * @param ep    Endpoint Descriptor.
   */
  void EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto, const USB_ENDPOINT_DESCRIPTOR *ep);
  /**@}*/

  uint8_t AssignChannel(uint8_t channel_number, uint8_t network_number, bool background_scan);
  uint8_t CloseChannel(uint8_t channel_number);
  uint8_t ConfigureChannel(uint8_t channel_number, uint8_t network_number, uint8_t device_type, uint16_t channel_period, uint8_t rf_freq, DataInCallback callback);
  uint8_t EnableExtended(uint8_t enable);
  uint8_t OpenChannel(uint8_t channel_number, DataInCallback callback);
  uint8_t ResetSystem();
  uint8_t SetChannelFrequency(uint8_t channel_number, uint8_t frequency);
  uint8_t SetChannelId(uint8_t channel_number, uint16_t device_id, uint8_t device_type);
  uint8_t SetChannelPeriod(uint8_t channel_number, uint16_t period);
  uint8_t SetChannelSearchTimeout(uint8_t channel_number, uint8_t timeout);
  uint8_t SetLowPrioritySearchTimeout(uint8_t channel_number, uint8_t timeout);
  uint8_t SetNetworkKey(uint8_t network_number, uint64_t netkey);

protected:
  /**
   * Used to print the USB Endpoint Descriptor.
   * @param ep_ptr Pointer to USB Endpoint Descriptor.
   */
  void PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr);
  void PrintUsbDeviceDescriptor(const USB_DEVICE_DESCRIPTOR* udd);

  /** Pointer to USB class instance. */
  USB *usb_;
  /** Device address. */
  uint8_t device_address_;
  /** Endpoint info structure. */
  EpInfo ep_info_[ANT_MAX_ENDPOINTS];

  /** Total number of endpoints in the configuration. */
  uint8_t num_ep_;
  /** Next poll time based on poll interval taken from the USB descriptor. */
  uint32_t next_poll_time_;

private:
  void    Initialize(); // Set all variables, endpoint structs etc. to default values
  uint8_t AntEventTask();
  uint8_t CalculateChecksum(uint8_t* message, uint8_t message_size);
  uint8_t ProcessAntPayload(uint8_t* message, uint8_t message_size);
  uint8_t TxMessage(uint8_t* message, uint8_t message_size);

  uint8_t config_num_;
  uint8_t poll_interval_;
  bool    poll_enable_;
  bool    max_channels_;

  DataInCallback *callbacks_;
};

#endif
