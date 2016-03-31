using System;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using Microsoft.ServiceBus.Messaging;
using Newtonsoft.Json;

namespace AzBrowzer
{
    public partial class MainWindow : Window
    {
        // Parameters for connection to Azure IoT Suite
        private string _connectionString =
            "HostName=XXX.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=XXXXXXXXX";

        private EventHubClient _eventHubClient;
        private string _iotHubD2cEndpoint = "messages/events";

        //Default url for tracking view
        const string _defaultURL = "http://www.purecreation.ru/suainav/index.html";
        //Storage for last opened page
        private string _lastUrl = null; 

        public MainWindow()
        {
            //Initialization of GUI components
            InitializeComponent();

            //Navigate to the default page
            wbBrowser.Navigate(_defaultURL);
        }

        private void Start()
        {
            //Creating a connection to Event Hub
            _eventHubClient = EventHubClient.CreateFromConnectionString(_connectionString, _iotHubD2cEndpoint);

            // Retrieves information of available Event Hub partitions
            var d2cPartitions = _eventHubClient.GetRuntimeInformation().PartitionIds;

            //Start asyncronouse message receiving
            foreach (string partition in d2cPartitions)
            {
                ReceiveMessagesFromDeviceAsync(partition);
            }
        }

        /// <summary>
        /// Receiving messages from device in asyncronous mode
        /// </summary>
        /// <param name="partition">Partition ID</param>
        private async Task ReceiveMessagesFromDeviceAsync(string partition)
        {
            // Create receiver for the group
            var eventHubReceiver = _eventHubClient.GetDefaultConsumerGroup().CreateReceiver(partition, DateTime.UtcNow);

            while (true)
            {
                // Wait and receive event message
                EventData eventData = await eventHubReceiver.ReceiveAsync();
                if (eventData == null) continue;

                //Convert received bytes to string, containing data in JSON format
                string data = Encoding.UTF8.GetString(eventData.GetBytes());

                // Store message text in log storage
                tbLog.Text += string.Format("Message received. Partition: {0} Data: '{1}'\n", partition, data);

                //Deserializing JSON data to special container class VehicleNavigationMessage
                VehicleNavigationMessage deserializedProduct = JsonConvert.DeserializeObject<VehicleNavigationMessage>(data);

                //If received URL was empty, then vehicle is out of the tracking sensors range, show default tracking page
                if (string.IsNullOrEmpty(deserializedProduct.Url))
                {
                    tbLog.Text += "EMPTY URL\n";

                    deserializedProduct.Url = _defaultURL;
                }

                //If received URL is not equal to last URL, then we need to refresh tracking screen to the new vehicle location
                if (deserializedProduct.Url != _lastUrl)
                {
                    wbBrowser.Navigate(deserializedProduct.Url);
                    _lastUrl = deserializedProduct.Url;
                }
            }
        }

        private void ButtonBase_OnClick(object sender, RoutedEventArgs e)
        {
            // Start receiving messages
            Start();
        }

        private void MainWindow_OnLoaded(object sender, RoutedEventArgs e)
        {
            // After GUI loaded, start receiving messages
            Start();
        }
    }
}
