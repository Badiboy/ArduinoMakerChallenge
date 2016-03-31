using System.Windows;
using System.Windows.Media;

namespace DeviceIndicator
{ 
    public partial class LEDControl
    {
        /// <summary>Dependency property to Get/Set the current IsActive (True/False)</summary>
        public static readonly DependencyProperty IsActiveProperty =
            DependencyProperty.Register("IsActive", typeof(bool), typeof(LEDControl),
                new PropertyMetadata(false, IsActivePropertyChanced));

        public static readonly DependencyProperty ColorOnProperty =
           DependencyProperty.Register("ColorOn", typeof(ImageSource), typeof(LEDControl),
               new PropertyMetadata(null, OnColorOnPropertyChanged));

        public static readonly DependencyProperty ColorOffProperty =
         DependencyProperty.Register("ColorOff", typeof(ImageSource), typeof(LEDControl),
             new PropertyMetadata(null, OnColorOffPropertyChanged));
         
        /// <summary>Gets/Sets Value</summary>
        public bool IsActive
        {
            get { return (bool)GetValue(IsActiveProperty); }
            set
            {
                SetValue(IsActiveProperty, value);
            }
        }

        public ImageSource ColorOn
        {
            get
            {
                return (ImageSource)GetValue(ColorOnProperty);
            }
            set
            {
                SetValue(ColorOnProperty, value);
            }
        }

        public ImageSource ColorOff
        {
            get
            {
                return (ImageSource)GetValue(ColorOffProperty);
            }
            set
            {
                SetValue(ColorOffProperty, value);
            }
        }
         
        public LEDControl()
        {
            InitializeComponent();

            if (IsActive == true)
                imgLed.Source = ColorOn;
            else
                imgLed.Source = ColorOff;
            //else
            //    backgroundColor.Color = ColorNull;
        }

        private static void IsActivePropertyChanced(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            LEDControl led = (LEDControl)d;
            if (led.IsActive == true)
                led.imgLed.Source = led.ColorOn;
            else
                led.imgLed.Source = led.ColorOff;

        }

        private static void OnColorOnPropertyChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            LEDControl led = (LEDControl)d;
            led.ColorOn = (ImageSource)e.NewValue;
            if (led.IsActive == true)
                led.imgLed.Source = led.ColorOn;
        }

        private static void OnColorOffPropertyChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            LEDControl led = (LEDControl)d;
            led.ColorOff = (ImageSource)e.NewValue;
            if (led.IsActive == false)
                led.imgLed.Source = led.ColorOff;
        }
    }
}
