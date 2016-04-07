#include "ant.h"

#define MSG_ASSIGN_CHANNEL   0x42
#define MSG_BROADCAST_DATA   0x4E
#define MSG_CHANNEL_FREQ     0x45
#define MSG_CHANNEL_ID       0x51
#define MSG_CHANNEL_PERIOD   0x43
#define MSG_CHANNEL_RESPONSE 0x40
#define MSG_CHANNEL_SEARCH_TIMEOUT 0x44
#define MSG_CLOSE_CHANNEL    0x4C
#define MSG_ENABLE_EXTENDED  0x66
#define MSG_LOW_PRIORITY_SEARCH_TIMEOUT 0X63
#define MSG_OPEN_CHANNEL     0X4B
#define MSG_RESET_SYSTEM     0x4A
#define MSG_SEARCH_TIMEOUT   0x44
#define MSG_SET_NETWORK_KEY  0x46

Ant::Ant(USB *p) :
usb_(p), // Pointer to USB class instance - mandatory
device_address_(0), // Device address - mandatory
num_ep_(1), // If config descriptor needs to be parsed
next_poll_time_(0), // Reset NextPollTime
poll_interval_(0),
poll_enable_(false), // Don't start polling before dongle is connected
max_channels_(3)
{
  Initialize(); // Set all variables, endpoint structs etc. to default values   

  if(usb_) // Register in USB subsystem
    usb_->RegisterDeviceClass(this); // Set devConfig[] entry

  callbacks_ = (void(**)(uint8_t*, uint16_t)) calloc(max_channels_, sizeof(DataInCallback));
}

uint8_t Ant::ConfigureDevice(uint8_t parent, uint8_t port, bool lowspeed) {
#ifdef EXTRADEBUG
  Notify(PSTR("\r\n*** entering Ant::ConfigureDevice ***"), 0x80);
#endif

  const uint8_t kBufSize = sizeof (USB_DEVICE_DESCRIPTOR);
  uint8_t buf[kBufSize];
  USB_DEVICE_DESCRIPTOR *udd = reinterpret_cast<USB_DEVICE_DESCRIPTOR*>(buf);
  uint8_t rcode;
  UsbDevice *p = NULL;
  EpInfo *oldep_ptr = NULL;

  Initialize(); // Set all variables, endpoint structs etc. to default values

  // Fill out control endpoint in UsbDevice structure.
  AddressPool &addr_pool = usb_->GetAddressPool();
  // Check if address has already been assigned to an instance
  if(device_address_) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nAddress in use"), 0x80);
#endif
    return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
  }

  // Get pointer to pseudo device with address 0 assigned
  p = addr_pool.GetUsbDevicePtr(0);
  if(!p) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nAddress not found"), 0x80);
#endif
    return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
  }

  if(!p->epinfo) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nepinfo is null"), 0x80);
#endif
    return USB_ERROR_EPINFO_IS_NULL;
  }

  oldep_ptr = p->epinfo; // Save old pointer to EP_RECORD of address 0
  // Temporary assign new pointer to ep_info_ to p->epinfo in order to avoid
  // toggle inconsistence.
  p->epinfo = ep_info_;
  p->lowspeed = lowspeed;
  // Get device descriptor - addr, ep, nbytes, data
  rcode = usb_->getDevDescr(0, 0, kBufSize, (uint8_t*)buf);
  p->epinfo = oldep_ptr; // Restore p->epinfo

  if(rcode)
    goto FailGetDevDescr;

#ifdef EXTRADEBUG
  PrintUsbDeviceDescriptor(udd);
#endif

  // Allocate new address according to device class
  device_address_ = addr_pool.AllocAddress(parent, false, port);

  if(!device_address_) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nOut of address space"), 0x80);
#endif
    return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;
  }

  // Extract Max Packet Size from device descriptor
  ep_info_[0].maxPktSize = udd->bMaxPacketSize0;

  return USB_ERROR_CONFIG_REQUIRES_ADDITIONAL_RESET;

FailGetDevDescr:
#ifdef DEBUG_USB_HOST
  NotifyFailGetDevDescr(rcode);
#endif
  if(rcode != hrJERR)
    rcode = USB_ERROR_FailGetDevDescr;
    Release();
  return rcode;
};

uint8_t Ant::Init(uint8_t parent, uint8_t port, bool lowspeed) {
#ifdef EXTRADEBUG
  Notify(PSTR("\r\n*** entering Ant::Init ***"), 0x80);
#endif
  uint8_t rcode;
  ConfigDescParser<0xFF, 0, 0, CP_MASK_COMPARE_ALL> confDescrParser(this);

  AddressPool &addr_pool = usb_->GetAddressPool();
  // Get pointer to assigned address record
  UsbDevice *p = addr_pool.GetUsbDevicePtr(device_address_);

  if(!p) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nAddress not found"), 0x80);
#endif
    return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
  }

  // Assign new address to the device
  rcode = usb_->setAddr(0, 0, device_address_);
  if(rcode) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nsetAddr: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
#endif
    p->lowspeed = false;
    goto Fail;
  }
#ifdef EXTRADEBUG
  Notify(PSTR("\r\nAddr: "), 0x80);
  D_PrintHex<uint8_t > (device_address_, 0x80);
#endif

  p->lowspeed = lowspeed;

   // Assign ep_info_ to epinfo pointer - only EP0 is known
  rcode = usb_->setEpInfoEntry(device_address_, 1, ep_info_);
  if(rcode)
    goto FailSetDevTblEntry;
      
  rcode = usb_->getConfDescr(device_address_, 0, 0, &confDescrParser);

  if(rcode)
    goto FailGetConfDescr;

  rcode = usb_->setEpInfoEntry(device_address_, num_ep_, ep_info_);
  if(rcode)
    goto FailSetDevTblEntry;

  // Set Configuration Value
  rcode = usb_->setConf(device_address_, ep_info_[ ANT_CTRL_PIPE ].epAddr,
      config_num_);
  if(rcode)
      goto FailSetConfDescr;

  poll_enable_ = true;

#ifdef EXTRADEBUG
  Notify(PSTR("\r\nAnt Dongle Initialized"), 0x80);
#endif
  return 0; // Successful configuration

        /* Diagnostic messages */
FailSetDevTblEntry:
#ifdef DEBUG_USB_HOST
  NotifyFailSetDevTblEntry();
  goto Fail;
#endif

FailGetConfDescr:
#ifdef DEBUG_USB_HOST
  NotifyFailGetConfDescr();
  goto Fail;
#endif

FailSetConfDescr:
#ifdef DEBUG_USB_HOST
  NotifyFailSetConfDescr();
  goto Fail;
#endif

Fail:
#ifdef DEBUG_USB_HOST
  Notify(PSTR("\r\nAnt Init Failed, error code: "), 0x80);
  NotifyFail(rcode);
#endif
  Release();
  return rcode;
}

void Ant::Initialize() {
  uint8_t i;
  for(i = 0; i < ANT_MAX_ENDPOINTS; i++) {
    ep_info_[i].epAddr = 0;
    ep_info_[i].maxPktSize = (i) ? 0 : 8;
    ep_info_[i].bmSndToggle = 0;
    ep_info_[i].bmRcvToggle = 0;
    ep_info_[i].bmNakPower = (i) ? USB_NAK_NOWAIT :
        USB_NAK_MAX_POWER;
  }

  device_address_ = 0; // Clear device address
  num_ep_ = 1; // Must have to be reset to 1
  next_poll_time_ = 0; // Reset next poll time
  poll_interval_ = 0;
  poll_enable_ = false; // Don't start polling before dongle is connected
}

/* Extracts bulk-IN, bulk-OUT endpoint information from config descriptor */
void Ant::EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto,
    const USB_ENDPOINT_DESCRIPTOR *pep) {
  uint8_t index;

#ifdef EXTRADEBUG
  PrintEndpointDescriptor(pep);
#endif

  if((pep->bmAttributes & 0x02) == 2) // Bulk endpoint found
    index = ((pep->bEndpointAddress & 0x80) == 0x80) ?
        ANT_DATA_IN_PIPE : ANT_DATA_OUT_PIPE;
  else
    return;

  // Fill the rest of endpoint data structure
  ep_info_[index].epAddr = (pep->bEndpointAddress & 0x0F);
  ep_info_[index].maxPktSize = (uint8_t)pep->wMaxPacketSize;
  // Set the polling interval as the largest polling interval obtained
  // from endpoint.
  if(poll_interval_ < pep->bInterval)
    poll_interval_ = pep->bInterval;
  num_ep_++;
}

void Ant::PrintUsbDeviceDescriptor(const USB_DEVICE_DESCRIPTOR* udd) {
#ifdef EXTRADEBUG
  Notify(PSTR("\r\nbLength:\t\t"), 0x80);
  D_PrintHex<uint8_t > (udd->bLength, 0x80);
  Notify(PSTR("\r\nbDescriptorType:\t\t"), 0x80);
  D_PrintHex<uint8_t > (udd->bDescriptorType, 0x80);
  Notify(PSTR("\r\nbcdUSB:\t"), 0x80);
  D_PrintHex<uint16_t > (udd->bcdUSB, 0x80);
  Notify(PSTR("\r\nbDeviceClass:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->bDeviceClass, 0x80);
  Notify(PSTR("\r\nbDeviceSubClass:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->bDeviceSubClass, 0x80);
  Notify(PSTR("\r\nbDeviceProtocol:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->bDeviceProtocol, 0x80);
  Notify(PSTR("\r\nbMaxPacketSize:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->bMaxPacketSize0, 0x80);
  Notify(PSTR("\r\nidVendor:\t"), 0x80);
  D_PrintHex<uint16_t > (udd->idVendor, 0x80);
  Notify(PSTR("\r\nidProduct:\t"), 0x80);
  D_PrintHex<uint16_t > (udd->idProduct, 0x80);
  Notify(PSTR("\r\nbcdDevice:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->bcdDevice, 0x80);
  Notify(PSTR("\r\niManufacturer:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->iManufacturer, 0x80);
  Notify(PSTR("\r\niProduct:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->iProduct, 0x80);
  Notify(PSTR("\r\niSerialNumber:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->iSerialNumber, 0x80);
  Notify(PSTR("\r\nbNumConfigurations:\t"), 0x80);
  D_PrintHex<uint8_t > (udd->bNumConfigurations, 0x80);
#endif
}

void Ant::PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr) {
#ifdef EXTRADEBUG
  Notify(PSTR("\r\nEndpoint descriptor:"), 0x80);
  Notify(PSTR("\r\nLength:\t\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bLength, 0x80);
  Notify(PSTR("\r\nType:\t\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bDescriptorType, 0x80);
  Notify(PSTR("\r\nAddress:\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bEndpointAddress, 0x80);
  Notify(PSTR("\r\nAttributes:\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bmAttributes, 0x80);
  Notify(PSTR("\r\nMaxPktSize:\t"), 0x80);
  D_PrintHex<uint16_t > (ep_ptr->wMaxPacketSize, 0x80);
  Notify(PSTR("\r\nPoll Intrv:\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bInterval, 0x80);
#endif
}

/* Performs a cleanup after failed Init() attempt */
uint8_t Ant::Release() {
  Initialize(); // Set all variables, endpoint structs etc. to default values
  usb_->GetAddressPool().FreeAddress(device_address_);
  return 0;
}

uint8_t Ant::Poll() {
  // Don't poll if shorter than polling interval
  if(poll_enable_ && (long)(millis() - next_poll_time_) >= 0L) { 
    next_poll_time_ = millis() + poll_interval_; // Set new poll time
    return AntEventTask();
  }
  return 0;
}

uint8_t Ant::AntEventTask() {
  uint16_t maxPktSize = ep_info_[ANT_DATA_IN_PIPE].maxPktSize;
  uint8_t buffer[maxPktSize];
  uint16_t bytes = maxPktSize;
  uint8_t message[maxPktSize * 2];
  uint16_t message_write_pos = 0;
  uint16_t bytes_remaining = 0xFFFF;
  
  uint8_t rcode = usb_->inTransfer(device_address_,
      ep_info_[ANT_DATA_IN_PIPE].epAddr, &bytes, buffer);

  if(!rcode || rcode == hrNAK) { // Check for errors
    if (bytes > 0) {
      for (uint16_t i = 0; i < bytes; i++) {
        if (message_write_pos == 0) {
          // expect sync byte
        } else if (message_write_pos == 1) {
          bytes_remaining = buffer[i] + 2;
        } else {
          bytes_remaining--;
        }

        message[message_write_pos] = buffer[i];
        message_write_pos++;

        if (bytes_remaining == 0) {  // message complete
          // validate checksum
          uint8_t checksum = CalculateChecksum(message, message_write_pos - 1);
          if (message[message_write_pos - 1] == checksum ) {
            ProcessAntPayload(message + 2, message_write_pos - 3);
#ifdef EXTRADEBUG
          } else {
            Notify(PSTR("\r\nBad Checksum: expected "), 0x80);
            D_PrintHex<uint8_t > (checksum, 0x80);
            Notify(PSTR(" got "), 0x80);
            D_PrintHex<uint8_t > (message[message_write_pos - 1], 0x80);
#endif
          }
          message_write_pos = 0;
          bytes_remaining = 0xFFFF;
        }
      }
    }
    return 0;
  }
#ifdef DEBUG_USB_HOST
  else {
    Notify(PSTR("\r\nAnt event error: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
  }
#endif
  return rcode;
}

uint8_t Ant::ProcessAntPayload(uint8_t* message, uint8_t message_size) {
  switch (message[0]) {
    case MSG_BROADCAST_DATA:
#ifdef EXTRADEBUG
      Serial.print("\r\nrx ch ");
      Serial.print(message[1], DEC);
      Serial.print(":\t");
      int i;
      for (i = 2; i < message_size; i++) {
        Serial.print(message[i] >> 4, HEX);
        Serial.print(message[i] & 0x0f, HEX);
      }
#endif
      callbacks_[message[1]](message + 2, message_size - 2);
      break;
    case MSG_CHANNEL_RESPONSE:
      if (!message[3]) return 0;
#ifdef EXTRADEBUG
      Notify(PSTR("\r\nstatus:\tcommand "), 0x80);
      D_PrintHex<uint8_t > (message[2], 0x80);
      Notify(PSTR("; channel "), 0x80);
      D_PrintHex<uint8_t > (message[1], 0x80);
      Notify(PSTR("; code "), 0x80);
      D_PrintHex<uint8_t > (message[3], 0x80);
#endif
      break;
  }
  return 0;
}

uint8_t Ant::SetNetworkKey(uint8_t network_number, uint64_t netkey) {
  uint8_t msgbuf[10];
  msgbuf[0] = MSG_SET_NETWORK_KEY;
  msgbuf[1] = network_number;

  uint8_t *bytes = (uint8_t*) &netkey;
  msgbuf[2] = bytes[7];
  msgbuf[3] = bytes[6];
  msgbuf[4] = bytes[5];
  msgbuf[5] = bytes[4];
  msgbuf[6] = bytes[3];
  msgbuf[7] = bytes[2];
  msgbuf[8] = bytes[1];
  msgbuf[9] = bytes[0];
  
  uint8_t rcode =TxMessage(msgbuf, sizeof(msgbuf));
#ifdef DEBUG_USB_HOST
  if(rcode) {
    Notify(PSTR("\r\nAnt error setting netkey: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
  }
#endif
  return rcode;
}

uint8_t Ant::EnableExtended(uint8_t enable) {
  uint8_t enable_extended[] = { MSG_ENABLE_EXTENDED, 0, enable};
  uint8_t rcode = TxMessage(enable_extended, sizeof(enable_extended));
#ifdef DEBUG_USB_HOST
  if(rcode) {
    Notify(PSTR("\r\nAnt error enabling extended: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
  }
#endif
  return rcode;
}

uint8_t Ant::AssignChannel(uint8_t channel_number, uint8_t network_number,
    bool background_scan) {
  uint8_t assign_channel[] = { MSG_ASSIGN_CHANNEL, channel_number, 0x40, network_number, 0x01 };
  uint8_t rcode = TxMessage(assign_channel, background_scan ? sizeof(assign_channel) :
      sizeof(assign_channel) - 1);  
#ifdef DEBUG_USB_HOST
  if (rcode) {
    Notify(PSTR("\r\nAnt error assigning channel: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif
  return rcode;      
}

uint8_t Ant::SetChannelId(uint8_t channel_number, uint16_t device_id, uint8_t device_type) {
  uint8_t set_id[6];
  set_id[0] = MSG_CHANNEL_ID;
  set_id[1] = channel_number;

  uint8_t *bytes = (uint8_t*) &device_id;
  set_id[2] = bytes[1];
  set_id[3] = bytes[0];

  set_id[4] = device_type;
  set_id[5] = 0;
  uint8_t rcode = TxMessage(set_id, sizeof(set_id));
#ifdef DEBUG_USB_HOST
  if(rcode) {
    Notify(PSTR("\r\nAnt error setting ID: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif
  return rcode;  
}

uint8_t Ant::SetChannelPeriod(uint8_t channel_number, uint16_t period) {
  uint8_t set_period[4];
  set_period[0] = MSG_CHANNEL_PERIOD;
  set_period[1] = channel_number;
  uint8_t *bytes = (uint8_t*) &period;
  set_period[2] = bytes[0];
  set_period[3] = bytes[1];
  uint8_t rcode = TxMessage(set_period, sizeof(set_period));
#ifdef DEBUG_USB_HOST
  if(rcode) {
    Notify(PSTR("\r\nAnt error setting period: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif
  return rcode;
}

uint8_t Ant::SetLowPrioritySearchTimeout(uint8_t channel_number, uint8_t timeout) {
  uint8_t set_low_priority_search_timeout[] = { MSG_LOW_PRIORITY_SEARCH_TIMEOUT, channel_number, timeout };
  uint8_t rcode = TxMessage(set_low_priority_search_timeout, sizeof(set_low_priority_search_timeout));
#ifdef DEBUG_USB_HOST
  if(rcode) {
    Notify(PSTR("\r\nAnt error setting low priority search timeout: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif
  return rcode;
}

uint8_t Ant::SetChannelSearchTimeout(uint8_t channel_number, uint8_t timeout) {
  uint8_t set_channel_search_timeout[] = { MSG_CHANNEL_SEARCH_TIMEOUT, channel_number, 0 };
  uint8_t rcode = TxMessage(set_channel_search_timeout, sizeof(set_channel_search_timeout));
#ifdef DEBUG_USB_HOST
  if(rcode) {
    Notify(PSTR("\r\nAnt error setting channel search timeout: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif
  return rcode;
}

uint8_t Ant::SetChannelFrequency(uint8_t channel_number, uint8_t frequency) {
  uint8_t set_rf_freq[] = { MSG_CHANNEL_FREQ, channel_number, frequency };
  uint8_t rcode = TxMessage(set_rf_freq, sizeof(set_rf_freq));
#ifdef DEBUG_USB_HOST
  if(rcode) {
    Notify(PSTR("\r\nAnt error setting RF frequency: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif
  return rcode;  
}

uint8_t Ant::OpenChannel(uint8_t channel_number, DataInCallback callback) {
  uint8_t open_channel[] = { MSG_OPEN_CHANNEL, channel_number };
  uint8_t rcode = TxMessage(open_channel, sizeof(open_channel)); 
#ifdef EXTRADEBUG
  if(rcode) {
    Notify(PSTR("\r\nAnt error opening channel: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif

  callbacks_[channel_number] = callback;
  return rcode;
}

uint8_t Ant::CloseChannel(uint8_t channel_number) {
  uint8_t close_channel[] = { MSG_CLOSE_CHANNEL, channel_number };
  uint8_t rcode = TxMessage(close_channel, sizeof(close_channel)); 
#ifdef EXTRADEBUG
  if(rcode) {
    Notify(PSTR("\r\nAnt error closing channel: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif
  return rcode;
}

uint8_t Ant::ResetSystem() {
  uint8_t reset_system[] = { MSG_RESET_SYSTEM, 0 };
  uint8_t rcode = TxMessage(reset_system, sizeof(reset_system)); 
#ifdef EXTRADEBUG
  if(rcode) {
    Notify(PSTR("\r\nAnt error resetting system: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
    return rcode;
  }
#endif
  return rcode;
}

uint8_t Ant::TxMessage(uint8_t* message, uint8_t message_size) {
  uint8_t tx_buffer[message_size + 3];  // message plus sync, size and checksum

  tx_buffer[0] = 0xA4;                      // sync byte
  tx_buffer[1] = (uint8_t) message_size - 1; // message size - command size

  for(uint8_t i = 0; i < message_size; i++)
    tx_buffer[2+i] = message[i];

  // calculate the checksum
  for(uint8_t i = 0; i < sizeof(tx_buffer) - 1; i++)
    tx_buffer[sizeof(tx_buffer) - 1] =
        CalculateChecksum(tx_buffer, sizeof(tx_buffer) - 1);

#ifdef EXTRADEBUG
  Notify(PSTR("\r\ntx:\t"), 0x80);  
  for(uint8_t i = 0; i < sizeof(tx_buffer); i++)
    D_PrintHex<uint8_t > (tx_buffer[i], 0x80);
#endif

  uint8_t rcode = usb_->outTransfer(device_address_, ep_info_[ ANT_DATA_OUT_PIPE ].epAddr,
      sizeof(tx_buffer), tx_buffer);
      
  delay(250);
  return rcode;
}

uint8_t Ant::CalculateChecksum(uint8_t* message, uint8_t message_size) {
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < message_size; i++)
    checksum = checksum ^ message[i];
  return checksum;
}
