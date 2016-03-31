using InTheHand.Net.Sockets;

namespace DeviceIndicator
{
    // Class representing device with a bluetooth adapter
    public class Device
    {
        // Device identification string
        public string DeviceName { get; set; }

        public Device(BluetoothDeviceInfo device_info)
        {
            DeviceName = device_info.DeviceName;
        }

        public override string ToString()
        {
            return DeviceName;
        }
    }
}
