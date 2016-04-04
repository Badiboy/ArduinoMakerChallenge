using System;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using Microsoft.Azure.Devices.Client;
using Newtonsoft.Json;

namespace DeviceIndicator
{
    /// <summary>
    /// A class responsible for sending messages from active device to the cloud
    /// </summary>
    class Send
    {
        // The address of Azure IoT (Internet of Things) Hub
        private const string IotHubUri = "xxx.azure-devices.net";

        // The name of first device (????)
        private const string DeviceId1 = "BluetoothAtDevise3";

        // The password to the first device
        private const string DeviceKey1 = "xxxxxxxxxxxxxxxxxxxxxxxx";

        // URL of a web-page depicting location of first device 
        private const string DeviceNavUrl1 = "http://www.purecreation.ru/suainav/index.html?&dst=2203";

        // The name of second device (????)
        private const string DeviceId2 = "BluetoothAtVic";

        // The password to the second device
        private const string DeviceKey2 = "xxxxxxxxxxxxxxxxxxxxxxxx";

        // URL of a web-page depicting location of second device 
        private const string DeviceNavUrl2 = "http://www.purecreation.ru/suainav/index.html?&dst=5233";

        // Contains methods that a device can use to send messages to and receive from the service
        private static DeviceClient _deviceClient;

        // Number of current device (1 or 2)
        private static int _devNum;

        // The value showing if last message was from active device or not
        private static bool _lastVal = false;

        // Parameters for connection to Azure IoT Suite for the first device
        private static string Device1ConnectionString
        {
            get
            {
                return
                    "HostName=" + IotHubUri + ";" +
                    "DeviceId=" + DeviceId1 + ";" +
                    "SharedAccessKey=" + DeviceKey1;
            }
        }

        // Parameters for connection to Azure IoT Suite for the second device
        private static string Device2ConnectionString
        {
            get
            {
                return
                    "HostName=" + IotHubUri + ";" +
                    "DeviceId=" + DeviceId2 + ";" +
                    "SharedAccessKey=" + DeviceKey2;
            }
        }

        //Initialization of device client, responsible for sending messages to service
        public static void InitDeviceClient(int devNum)
        {
            _devNum = devNum;

            // Depending of device number, use initialization parameters for first or second device
            if (devNum == 1)
            {
                _deviceClient = DeviceClient.CreateFromConnectionString(Device1ConnectionString, TransportType.Http1);
            }
            else if (devNum == 2)
            {
                _deviceClient = DeviceClient.CreateFromConnectionString(Device2ConnectionString, TransportType.Http1);
            }
        }

        // Send message containing information about vehicle status - active or non-active
        public static async void SendMessage(bool msg)
        {
            try
            {
                if (_devNum == 1)
                {
                    await SendDeviceLocationMessage(DeviceId1, msg);
                }
                else if (_devNum == 2)
                {
                    await SendDeviceLocationMessage(DeviceId2, msg);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Sending message error:\n" + ex.Message);
            }
        }

        static async Task SendMessage(DeviceClient deviceClient, object data)
        {
            // Serializing message data to JSON format
            var messageString = JsonConvert.SerializeObject(data);

            // Convert message to the set of bytes and pass it to the Message class
            var message = new Message(Encoding.ASCII.GetBytes(messageString));

            // Send message asynchronously
            await deviceClient.SendEventAsync(message);
        }

        static async Task SendDeviceLocationMessage(string deviceId, bool msg)
        {
            // If status of vehicle changed, generate additional message
            // It is required for better plot shape
            if (msg != _lastVal)
            {
                var dataPoint1 = CreateMessage(deviceId, _lastVal);
                await SendMessage(_deviceClient, dataPoint1);

                _lastVal = msg;
            }

            // Create message with current vehicle status
            var dataPoint2 = CreateMessage(deviceId, msg);

            await SendMessage(_deviceClient, dataPoint2);
        }

        static VehicleNavigationMessage CreateMessage(string deviceId, bool msg)
        {
            //Create message basing on device identification string and its status 
            var result = new VehicleNavigationMessage(deviceId);

            int defaultHumidity = 110;

            //Choose correct web page address for selected device
            string url = deviceId == DeviceId1 ? DeviceNavUrl1 : DeviceNavUrl2;

            // If status is active, fill message fields with data
            if (msg)
            {
                result.URL = url;
                result.Humidity = defaultHumidity;
            }

            return result;
        }
    }
}
