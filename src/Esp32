#include <WiFi.h>
#include <FirebaseESP32.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <DHT.h>

#define WIFI_USERNAME   "100.000USDorLoser"
#define WIFI_PASSWORD   "huylong1999"
#define FIREBASE_HOST   "https://esp32-12371-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH   "Nv6h0vZhJ3inDzJLvnHieGssXKyxnMIbBARuURfp"
/* defines the pins used by the tranceiver module */
                                  // in my experience, this's the best interrupt pin on ESP32

#define FIREBASE_SENSOR_PATH     "/path_2/sensor"
#define FIREBASE_CONTROL_PATH "/path_1/control"
#define FIREBASE_STATUS_PATH     "/path_3/control_status"

/* RELAY PIN */
#define DEVICE_1 2
#define DEVICE_2 4
#define DEVICE_3 5
#define DEVICE_4 13

/* Sensor pin */
#define DHT11_PIN 32
#define BRIGHT_PIN 34
#define MOISTURE_PIN 35
#define POWER_MOISTURE_PIN  16

#define LCD_I2C_ADDRESS 0x27
#define LCD_ROWS        4                                               // LCD size
#define LCD_COLUMNS     20
/* store relay state (in case hard reset) */
#define EEPROM_SIZE     1                                               // use 1 byte of EEPROM
#define EEPROM_ADDRESS  0x00

void init_pin_mode(){                                                   // set up pinmode here

    pinMode(DEVICE_1, OUTPUT);
    pinMode(DEVICE_2, OUTPUT);
    pinMode(DEVICE_3, OUTPUT);
    pinMode(DEVICE_4, OUTPUT);
    pinMode(POWER_MOISTURE_PIN, OUTPUT);
}

void connect_eps32_wifi();

/* lcd function */
void lcd_setup();
void lcd_print_relay_state(uint8_t relay_byte);
void lcd_print_sensor_value(String *sensor_array);

/* analyse data, check data */
bool analyse_control_data(uint8_t control_byte, uint8_t *state_byte);
String read_sensor_data();

/* Define FirebaseESP32 data object for data sending and receiving */
FirebaseData fbdo;
/* Lcd initialize */
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
/* Init dht11 sensor */
DHT dht(DHT11_PIN, DHT11);

uint8_t relay_state = 0;                                                // store relay state
float sensor_data[4];


void setup() {

    dht.begin();
    lcd_setup();
    delay(1000);
    EEPROM.begin(EEPROM_SIZE);                                          // get relays's state from EEPROM
    relay_state = (EEPROM.read(EEPROM_ADDRESS)) & 0b00001111;
    init_pin_mode();
    lcd_print_and_control_relay(relay_state);
    delay(1000);
     Serial.begin(115200);
    connect_eps32_wifi();
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    /* Uncomment below line to enable syncworld - avoid signal from other LoRa*/
    // LoRa.setSyncWord(0xF3);

    Firebase.reconnectWiFi(true);                                       // enable auto reconnect the WiFi when connection lost
}

void loop() {

    yield();                                                            // give time for CPU complete it's work
    if(wait_for_sensor(3000)){
        /* send sensor data to firebase, return TRUE if sending successed */

        if(Firebase.setString(fbdo, FIREBASE_SENSOR_PATH, read_sensor_data())){

            lcd_print_sensor_value();
        }
        else{                                                   // push data to firebase failed

            // Serial.println(fbdo.errorReason());              // Uncomment this line to know reason why push data fail
            return;
        }
    }

    /* GET DATA FROM FIREBASE AND SEND IT TO NODE THROUGH LORA********************************/

    // get and send data every 1 seconds (700ms wait + 300ms get data from firebase)
    else if(wait_for_relay(800)){

        /* return TRUE if get data successed, and data will be stored in database(fbdo) */
        if(Firebase.getString(fbdo, FIREBASE_CONTROL_PATH)){

            String control_data = fbdo.stringData();                           // get data from database
            /* analyse control_data, return false if data was not available */
            if(analyse_control_data(control_data.toInt(), &relay_state)){

                 Serial.println(relay_state);
                lcd_print_and_control_relay(relay_state);
                EEPROM.write(EEPROM_ADDRESS, relay_state);            // saved relay state
                EEPROM.commit();
                if(Firebase.setInt(fbdo, FIREBASE_STATUS_PATH, relay_state)){

                    delay(10);
                    yield();
                }
            }
            else{                                                   // data format is not available

                return;
            }
        }
    }
    else{                                                       // get data failed

        // Serial.println(fbdo.errorReason());                  // Uncomment this line to know reason get data fail
        return;
    }
}

/**********************************************************************************************************************/

/* ANALYSE DATA FUNCTION *******************************************************/

bool analyse_control_data(uint8_t control_byte, uint8_t *state_byte){

    /* control_byte format in decimal: <relay that has change><state you want to change>
        example: 41 mean user want to turn on fourth relay */
    uint8_t tenths = control_byte / 10;
    uint8_t units = control_byte % 10;
    uint8_t temp = *state_byte;
    if(tenths > 4 || tenths < 1 || (units != 0 && units != 1)){

        return false;                                                   // format data is not familiar
    }
    else{

        if(units == 1){

            temp |= (0x01 << (tenths - 1));                      // set relay bit to 1
        }
        else if(units == 0){

            temp &= ~(0x01 << (tenths - 1));                     // clear relay bit
        }
    }
    if(temp != *state_byte){

        *state_byte = temp;
        return true;
    }
    else{

        return false;
    }
}

String read_sensor_data(){

    sensor_data[0] = dht.readTemperature();
    sensor_data[1] = dht.readHumidity();

    static int times = 0;
    if(times == 0){
        digitalWrite(POWER_MOISTURE_PIN, HIGH);
        delay(1000);
        sensor_data[3] = analogRead(MOISTURE_PIN);
        sensor_data[2] = analogRead(BRIGHT_PIN);
        digitalWrite(POWER_MOISTURE_PIN, LOW);
        sensor_data[3] = ((4095 - sensor_data[3]) / (4095 - 2000)) * 100;
        sensor_data[2] = 100 - (sensor_data[2] / 40.95);
        times = 1;
    }
    else{
      times--;
    }
    analyse_special_case();

    String send_str = "";
    send_str += sensor_data[0];
    send_str += "*";
    send_str += sensor_data[1];
    send_str += "*";
    send_str += sensor_data[2];
    send_str += "*";
    send_str += sensor_data[3];
    send_str += "*";
    return send_str;
}

/* LCD SUB FUNCTION ************************************************************/

void lcd_setup(){

//    lcd.begin();                                                        // setup lcd
//    byte water[8] = {
//
//        0x00, 0x04, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E
//    };  lcd.createChar(1, water);
//
//    byte temperature[8]= {
//
//        0x03, 0x0B, 0x08, 0x1E, 0x08, 0x08, 0x0A, 0x04
//    }; lcd.createChar(2, temperature);
//
//
//    byte degree[8]= {
//
//        0x18, 0x18, 0x06, 0x09, 0x08, 0x08, 0x09, 0x06
//    }; lcd.createChar(3, degree);
//
//    lcd.backlight();
//    lcd.setCursor(0, 0);    lcd.print("R1:");                           // print name of relay and sensor
//    lcd.setCursor(0, 1);    lcd.print("R2:");
//    lcd.setCursor(0, 2);    lcd.print("R3:");
//    lcd.setCursor(0, 3);    lcd.print("R4:");
//    lcd.setCursor(7, 0);    lcd.write(2);
//    lcd.setCursor(14, 0);   lcd.write(3);
//    lcd.setCursor(7, 1);    lcd.write(1);
//    lcd.setCursor(14, 1);   lcd.print("%");
//    lcd.setCursor(7, 2);    lcd.print("BRI:");
//    lcd.setCursor(17, 2);   lcd.print("%");
//    lcd.setCursor(7, 3);    lcd.print("MOI:");
//    lcd.setCursor(17, 3);   lcd.print("%");

    byte WATER_SYMBOL[] = {0x00, 0x04, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E};
    byte DEGGREE_SYMBOL[] = {0x18, 0x18, 0x06, 0x09, 0x08, 0x08, 0x09, 0x06};
    byte TEMPERATURE_SYMBOL[] = {0x03, 0x0B, 0x08, 0x1E, 0x08, 0x08, 0x0A, 0x04};
    byte ON_SYMBOL[] = { 0x1F, 0x1F, 0x1F, 0x1F, 0x11, 0x11, 0x11, 0x1F};
    byte OFF_SYMBOL[] = { 0x1F, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F};

    lcd.begin();                                                         // setup lcd
    lcd.backlight();
    lcd.createChar(0, WATER_SYMBOL);                                    // create char
    lcd.createChar(1, DEGGREE_SYMBOL);
    lcd.createChar(2,TEMPERATURE_SYMBOL);
    lcd.createChar(3, ON_SYMBOL);
    lcd.createChar(4, OFF_SYMBOL);
    /* print header */
    lcd.setCursor(0, 0);
    lcd.print("Q2   B2   D3:  D4:");
    lcd.setCursor(0, 1);    lcd.write(2);       lcd.setCursor(8, 1);   lcd.write(1);
    lcd.setCursor(10, 1);    lcd.write(0);      lcd.setCursor(18, 1);   lcd.print("%");
    lcd.setCursor(0, 2);    lcd.print("Anh sang: "); lcd.setCursor(18, 2); lcd.print("%");
    lcd.setCursor(0, 3);    lcd.print("Do am dat:"); lcd.setCursor(18, 3); lcd.print("%");
}

void lcd_print_and_control_relay(uint8_t relay_byte){
/*
    lcd.setCursor(3, 0);                                                // RELAY 1 state
    if((relay_byte & 0x01) != 0x00){                                    // first LSB of relay_state byte
        lcd.print("ON ");
        digitalWrite(DEVICE_1, LOW);
    }
    else{
        lcd.print("OFF");
        digitalWrite(DEVICE_1, HIGH);
    }

    lcd.setCursor(3, 1);                                                // RELAY 2 state
    if((relay_byte & 0x02) != 0x00){                                    // second LSB of relay_state byte
        lcd.print("ON ");
        digitalWrite(DEVICE_2, LOW);

    }
    else{
        lcd.print("OFF");
        digitalWrite(DEVICE_2, HIGH);
    }

    lcd.setCursor(3, 2);                                                // RELAY 3 state
    if((relay_byte & 0x04) != 0x00){                                    // third LSB of relay_state byte
        lcd.print("ON ");
        digitalWrite(DEVICE_3, LOW);
    }
    else{
        lcd.print("OFF");
        digitalWrite(DEVICE_3, HIGH);
    }

    lcd.setCursor(3, 3);                                                // RELAY 4 state
    if((relay_byte & 0x08) != 0x00){                                    // fourth bit of relay_state byte
        lcd.print("ON ");
        digitalWrite(DEVICE_4, LOW);
    }
    else{
        lcd.print("OFF");
        digitalWrite(DEVICE_4, HIGH);
    }
/******************************************************************************************************************/
    lcd.setCursor(3, 0);                                                // RELAY 1 state
    if((relay_byte & 0x01) != 0x00){                                    // first LSB of relay_state byte
          lcd.write(3);
        digitalWrite(DEVICE_1, LOW);
    }
    else{
          lcd.write(4);
        digitalWrite(DEVICE_1, HIGH);
    }

      lcd.setCursor(8, 0);
    if((relay_byte & 0x02) != 0x00){                                    // second LSB of relay_state byte
          lcd.write(3);
        digitalWrite(DEVICE_2, LOW);

    }
    else{
        lcd.write(4);
        digitalWrite(DEVICE_2, HIGH);
    }

    lcd.setCursor(13, 0);
    if((relay_byte & 0x04) != 0x00){                                    // third LSB of relay_state byte
        lcd.write(3);
        digitalWrite(DEVICE_3, LOW);
    }
    else{
        lcd.write(4);
        digitalWrite(DEVICE_3, HIGH);
    }

//    lcd.setCursor(3, 3);                                                // RELAY 4 state
    lcd.setCursor(18, 0);
    if((relay_byte & 0x08) != 0x00){                                    // fourth bit of relay_state byte
        lcd.write(3);
        digitalWrite(DEVICE_4, LOW);
    }
    else{
        lcd.write(4);
        digitalWrite(DEVICE_4, HIGH);
    }
}

void lcd_print_sensor_value(){

//    lcd.setCursor(9, 0);   lcd.print(sensor_data[0]);       // temperature value
//    lcd.setCursor(9, 1);   lcd.print(sensor_data[1]);        // humidity value
//    lcd.setCursor(11, 2);  lcd.print(sensor_data[2]);       // air quality value
//    lcd.setCursor(11, 3);  lcd.print(sensor_data[3]);
    lcd.setCursor(3, 1);   lcd.print(sensor_data[0]);       // temperature value
    lcd.setCursor(13, 1);   lcd.print(sensor_data[1]);        // humidity value
    lcd.setCursor(10, 2);  lcd.print(sensor_data[2]);       // air quality value
    lcd.setCursor(10, 3);  lcd.print(sensor_data[3]);
}

void connect_eps32_wifi(){

    WiFi.begin(WIFI_USERNAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED){

        delay(200);
    }
}

bool wait_for_relay(unsigned long interval){                                  // wait for a time (you can do other work)

    static bool is_enabled_timing = false;
    /* in fact, millis function run over and over, so we need to keep this static to mark your start time */
    static unsigned long previous_millis = 0;
    unsigned long current_millis = millis();
    if(!is_enabled_timing){                                             // check if timer was start to timing before?

        is_enabled_timing = true;
        previous_millis = current_millis;                               // if yes, set previous millis and start couting
    }
    if(current_millis - previous_millis >= interval)                    // check if timing reached limit
    {

        is_enabled_timing = false;                                      // stop using timer
        return true;
    }
    return false;
}

bool wait_for_sensor(unsigned long interval){

   static bool is_enabled_timing = false;
    /* in fact, millis function run over and over, so we need to keep this static to mark your start time */
    static unsigned long previous_millis = 0;
    unsigned long current_millis = millis();
    if(!is_enabled_timing){                                             // check if timer was start to timing before?

        is_enabled_timing = true;
        previous_millis = current_millis;                               // if yes, set previous millis and start couting
    }
    if(current_millis - previous_millis >= interval)                    // check if timing reached limit
    {

        is_enabled_timing = false;                                      // stop using timer
        return true;
    }
    return false;
}

void analyse_special_case(){

   static bool is_temp_problem_solved = true;
   static bool is_moi_problem_solved = true;
   if(sensor_data[0] > 30 && digitalRead(DEVICE_1)){
         do{
             yield();
         }
         while(!Firebase.setString(fbdo, FIREBASE_CONTROL_PATH, "11"));
         is_temp_problem_solved = false;
    }
    else if(sensor_data[0] < 28 && !digitalRead(DEVICE_1) && !is_temp_problem_solved){

        do{
           yield();
        }
        while(!Firebase.setString(fbdo, FIREBASE_CONTROL_PATH, "10"));
        is_temp_problem_solved = true;
    }
    if(sensor_data[0] < 15 && !digitalRead(DEVICE_1)){
         do{
             yield();
         }
         while(!Firebase.setString(fbdo, FIREBASE_CONTROL_PATH, "10"));
    }

    if(sensor_data[3] < 20 && digitalRead(DEVICE_2)){
        do{
           yield();
        }
        while(!Firebase.setString(fbdo, FIREBASE_CONTROL_PATH, "21"));
        is_moi_problem_solved = false;
    }
    else if(sensor_data[3] > 23 && !digitalRead(DEVICE_2) && is_moi_problem_solved){

        do{
           yield();
        }
        while(!Firebase.setString(fbdo, FIREBASE_CONTROL_PATH, "20"));
        is_moi_problem_solved = true;
    }
    if(sensor_data[3] > 50 && !digitalRead(DEVICE_2)){
        do{
           yield();
        }
        while(!Firebase.setString(fbdo, FIREBASE_CONTROL_PATH, "20"));
    }
}
