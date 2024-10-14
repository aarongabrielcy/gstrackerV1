#include <Arduino.h>
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 44, TXPin = 43;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial serialgps(RXPin, TXPin);

XPowersPMU  PMU;

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

#define TINY_GSM_RX_BUFFER 1024

#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(Serial1, Serial);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

const char *register_info[] = {
    "Not registered, MT is not currently searching an operator to register to.The GPRS service is disabled, the UE is allowed to attach for GPRS if requested by the user.",
    "Registered, home network.",
    "Not registered, but MT is currently trying to attach or searching an operator to register to. The GPRS service is enabled, but an allowable PLMN is currently not available. The UE will start a GPRS attach as soon as an allowable PLMN is available.",
    "Registration denied, The GPRS service is disabled, the UE is not allowed to attach for GPRS if it is requested by the user.",
    "Unknown.",
    "Registered, roaming.",
};

enum {
    MODEM_CATM = 1,
    MODEM_NB_IOT,
    MODEM_CATM_NBIOT,
};

double latitude = 0.0, longitude = 0.0, speed_kmh = 0.0, course = 0.0;
unsigned long satellites = 0;
String dateTime, data, datetimeModem;
bool gnss_valid = false, gprs_state = false, modem_config_state = false, level = false;
int fix = 0;
unsigned long previousMillis = 0;
const long interval = 7000; // Intervalo de 1 segundo (1000 ms)

const char *apns[] = {
    "ott.iot.attmex.mx",
    "internet.itelcel.com"
};
String inputCommand = "", readcommand = "";

float  power_in    = 0.00;
float  backup_batt = 0.00;
bool powerOff = false;
bool powerOn = false;

const char* server = "34.196.135.179"; //V3 PROD
//const char* overwritten_imei = "6047403956";
const char* overwritten_imei = "6047417071";
const int port = 5200;
const char *headers[] = {
    "STT",
    "ALT",
    "RES",
    "CMD"
};
enum AlertId {
  IGN_ON    = 33, 
  IGN_OFF   = 34,   
  POWER_ON  = 40,
  POWER_OFF = 41,
  BATT_ON   = 44,
  BATT_OFF  = 45,
  IN_1_ON   = 11,
  IN_1_OFF  = 12,
  PANIC_BTN = 42,
};

int ign = 0, mode = 0, in1 = 0, in2 = 0, out1 = 0, out2 = 0;
const int GPO8  = 8; //Power in (sheld 1.1)
const int GPO9  = 9; //power in (back up sheld 1.1)
const int GPO10 = 46; // ign;rojo
const int GPO11 = 45;// in1;azul
const int GPO12 = 12; //in2
const int GPO13 = 13; //out1;amarillo
const int GPO14 = 14; //out2

const int R1_divisor_power_in = 4700;//we read the voltage acrosst this resistor (car resistors)
const int R2_divisor_power_in = 43000;
const int R1_divisor_batt = 5100;//we read the voltage acrosst this resistor (backup resistors)
const int R2_divisor_batt = 1800;

bool estadoPulsadorAnterior = HIGH;
bool LaststateIgnition = HIGH;
bool LaststateInput1 = HIGH;

void setup(){

  Serial.begin(9600);
  serialgps.begin(GPSBaud);
  while (!Serial);
  delay(3000);

  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    Serial.println("Failed to initialize power.....");
    while (1) {
      delay(5000);
    }
  }
  PMU.setBLDO1Voltage(3300);    //Set the power supply for level conversion to 3300mV
  PMU.enableBLDO1();

    //Set the working voltage of the modem, please do not modify the parameters
  PMU.setDC3Voltage(3000);    //SIM7080 Modem main power channel 2700~ 3400V
  PMU.enableDC3();

  pinMode(GPO13, OUTPUT);
  pinMode(GPO10,INPUT_PULLUP);
  pinMode(GPO11,INPUT_PULLUP);
  /*pinMode(GPO12,OUTPUT);
  digitalWrite(GPO12,HIGH);*/

  Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);
  modem_config_state = validateNetwork();
  if(modem_config_state){
    Serial.println("modem no configurado ##");
      configModem();
  }
  
}

void loop(){
  //Calculate Events
 mode = mode_divice();
  power_in = get_power_value();
  backup_batt = get_batt_value();
  // Emmits Events
  ignition_event();
  input_event();
  /*panic_event();
  power_conn_event(power_in);*/

  unsigned long currentMillis = millis();
  // This sketch displays information every time a new sentence is correctly encoded.
  while (serialgps.available() > 0){
    gps.encode(serialgps.read());
    
    if (gps.location.isUpdated()){
      gnss_valid = true;
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      speed_kmh = gps.speed.kmph();
      course = gps.course.deg();
      satellites = gps.satellites.value();
      fix = gps.location.isValid() ? 1 : 0;
      dateTime = getDateTimeGPS(gps.date, gps.time);
    }
  }
  if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        if(gnss_valid){
          data = String(headers[0])+";"+overwritten_imei+";3FFFFF;95;1.0.21;1;"+dateTime+";04BB4A02;334;20;3C1F;18;+"+String(latitude, 6)+";"+String(longitude, 6)+";"+speed_kmh+";"+course+";"+satellites+";"+fix+";00000"+in2+in1+ign+";000000"+out2+out1+";"+mode+";1;0929;"+backup_batt+";"+power_in;
          /*Serial.print("Latitude= "); 
          Serial.print(latitude, 6);      
          Serial.print(" Longitude= "); 
          Serial.println(longitude, 6);
          Serial.print("dateTime format = ");
          Serial.println(dateTime);
          Serial.print("Speed in km/h = "); 
          Serial.println(speed_kmh);
          Serial.print("Course in degrees = "); 
          Serial.println(course);
          // Number of satellites in use (u32)
          Serial.print("Number os satellites in use = "); 
          Serial.println(satellites);*/
        }else{
          Serial.println("SIN señal GNSS, mostrando valores por defecto:");
          getModemDateTime();
          data = String(headers[0])+";"+overwritten_imei+";3FFFFF;95;1.0.21;1;"+datetimeModem+";04BB4A02;334;20;3C1F;18;+"+String(latitude, 6)+";"+String(longitude, 6)+";"+speed_kmh+";"+course+";"+satellites+";"+fix+";00000"+in2+in1+ign+";000000"+out2+out1+";"+mode+";1;0929;"+backup_batt+";"+power_in;
        }
        Serial.println(data);
        sendDATA(data);
        gnss_valid = false;
    }
    if(!gprs_state){
      modem_config_state = configModem();
    }
}
void sendDATA(String data){
  TinyGsmClient client(modem, 0);
  client.print(data);

  if (client.available()) {
    Serial.println("Datos recibidos del servidor:");
    String response = "";  // Variable temporal para almacenar la respuesta completa
    
    while (client.available()) {
      char c = client.read();
      response += c;  // Leer y almacenar la respuesta completa
    }
    Serial.println("message => ");
    readcommand = response;
    Serial.println(readcommand);  
    if(output_active(readcommand) ){
        //client.print(myresponse);
    }else{
        //client.print(myresponse);
    }
    
  } else {
    Serial.println("No hay datos disponibles del servidor");
  }
  //despues de leerlo manda la respuesta al servidor "RSP"
}
bool output_active(String input){
  TinyGsmClient client(modem, 0);
  String response = String(headers[2])+";"+overwritten_imei+";"+dateTime+";000039C5;"+String(latitude, 6)+";"+String(longitude, 6)+";"+speed_kmh+";"+course+";"+satellites+";"+fix+";2958851;"+power_in+";00000"+in2+in1+ign+";000000"+out2+out1+";0;0";
  String seco_on = String(headers[3])+";"+overwritten_imei+";"+"04;01";
  String seco_off = String(headers[3])+";"+overwritten_imei+";"+"04;02";
  Serial.print("SECO =: ");
  Serial.println(seco_on);
  if(input == seco_on){
  
    PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
    digitalWrite(GPO13, HIGH);
    out1 = 1;
    client.print(response);
    Serial.println("Activar salida ===========>");
    return true;
  }else if(input == seco_off){
    PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    digitalWrite(GPO13, LOW);
    out1 = 0;
    client.print(response);
    Serial.println("Desactivar salida ===========>");      
  }else{
    Serial.println("Error al activar la salida!!");
  }
  return false;
}
void ignition_event(){
  int StateIgnition = digitalRead(GPO10);
  if (StateIgnition == LOW && LaststateIgnition == HIGH) {
    Serial.println("*** ¡ignition ON! **** ");
    ign = 1;
     event_generated(IGN_ON);
  }else if(StateIgnition == HIGH && LaststateIgnition == LOW){
    Serial.println("**** ¡ignition OFF! ***** ");
    ign = 0;
    event_generated(IGN_OFF);
  }
  LaststateIgnition = StateIgnition;
  //delay(200);
}
bool validateNetwork(){
  bool res = modem.isGprsConnected();
  Serial.print("GPRS status:");
  Serial.println(res ? "connected" : "not connected");
  //############################################################
  RegStatus s;
  do {
    s = modem.getRegistrationStatus();
    if (s != REG_OK_HOME && s != REG_OK_ROAMING) {
      Serial.print(".");
      PMU.setChargingLedMode(level ? XPOWERS_CHG_LED_ON : XPOWERS_CHG_LED_OFF);
      level ^= 1;
      delay(1000);
    }
  } while (s != REG_OK_HOME && s != REG_OK_ROAMING);
    Serial.print("Network register info:");
    Serial.println(register_info[s]);
  return res;
}
bool configModem(){
  TinyGsmClient client(modem, 0);

  //###################################################
  pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
  pinMode(BOARD_MODEM_DTR_PIN, OUTPUT);
  pinMode(BOARD_MODEM_RI_PIN, INPUT);

  int retry = 0;
  while (!modem.testAT(1000)) {
    Serial.print(".");
    if (retry++ > 6) {
      // Pull down PWRKEY for more than 1 second according to manual requirements
      digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
      delay(1000);
      digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
      retry = 0;
      Serial.println("Retry start modem .");
    }
  }
  Serial.println();
  Serial.print("Modem started!");
  //#####################################################
  do{
    //agregar parpadeo de led azul rapidos para indicar que no hay sim insertada
    Serial.println("SIM Card is not insert!!!");
  }while (modem.getSimStatus() != SIM_READY);
 //#######################################################
 modem.sendAT("+CFUN=1");
  if (modem.waitResponse(20000UL) == 1) {
    Serial.println("Enable RF Failed!");
  }
 //#######################################################
  modem.setNetworkMode(1);    //use automatic
  modem.setPreferredMode(MODEM_CATM);
  uint8_t pre = modem.getPreferredMode();
  uint8_t mode = modem.getNetworkMode();
  Serial.printf("getNetworkMode:%u getPreferredMode:%u\n", mode, pre);
  //#########################################################
  do{
    modem.sendAT("+CGDCONT=1,\"IP\",\"", apns[0], "\"");
    delay(2000);
  }while(modem.waitResponse() != 1);
  //##########################################################
  //!! Set the APN manually. Some operators need to set APN first when registering the network.
  modem.sendAT("+CNCFG=0,1,\"", apns[0], "\"");
  if (modem.waitResponse() != 1) {
    Serial.println("Config apn Failed!");
  }
  modem.sendAT("AT+CNCFG?");
  Serial.print("+CNCFG? => ");
  Serial.println(modem.waitResponse() );
  //#########################################################
  do{
    modem.sendAT("+CNACT=0,1");
    delay(2000);
    modem.sendAT("+CNACT?");
    Serial.print("+CNACT? => ");
    Serial.println(modem.waitResponse() );
  }while(modem.waitResponse() != 1);
  //###########################################################
  if (!client.connect(server, port)) {
    Serial.println("Error en la conexión al servidor");
  }
  //############################################################
  gprs_state = validateNetwork();
  return gprs_state;
}
float get_power_value(){
  uint32_t voltage_mV2 = analogReadMilliVolts(GPO8); // Read the voltage in millivolts
  int mV2 = voltage_mV2;
  float pwr = (((float) voltage_mV2) / 1000.0)  * (1 + (float)R2_divisor_power_in/(float)R1_divisor_power_in);
  return pwr;
}
float get_batt_value(){
  uint32_t voltage_mV1 = analogReadMilliVolts(GPO9);
  int mv1 = voltage_mV1;
  float batt = (((float) voltage_mV1) / 1000.0)  * (1 + (float)R2_divisor_batt/(float)R1_divisor_batt);
  return batt;
}
void panic_event(){
  int estadoPulsador = digitalRead(GPO11);
  if (estadoPulsador == LOW && estadoPulsadorAnterior == HIGH) {
    Serial.println("¡botón de pánico presionado!");
    event_generated(PANIC_BTN);
  }
  estadoPulsadorAnterior = estadoPulsador;
  delay(200);
}
void input_event(){
  int stateInput1 = digitalRead(GPO11);
  if (stateInput1 == LOW && LaststateInput1 == HIGH) {
    Serial.println("############ ¡input 1 ON! ##############");
    in1 = 1;
    event_generated(IN_1_ON);
  }else if(stateInput1 == HIGH && LaststateInput1 == LOW){
    Serial.println("############# input 1 OFF! ##############");
    in1 = 0;
    event_generated(IN_1_OFF);
  }
  LaststateInput1 = stateInput1;
  //delay(200); 
}

int output_event(){
  return 0;
}

void power_conn_event(float power){
  if(power < 1.5 && !powerOff){
    Serial.println("¡alimentación desconectada!");
    powerOff = true;
    powerOn  = false;
    event_generated(POWER_OFF);
    
  }else if(power > 10.1 && !powerOn){
    Serial.println("¡alimentación conectada!");
    powerOff = false;
    powerOn = true;
    event_generated(POWER_ON);
  }
}
bool battery_conn_event(){
  return false;
}
int mode_divice(){
  int mode_dev = 0;
    if(speed_kmh > 10  && ign == 0){
      return mode_dev = 0;
    }else if(speed_kmh > 10  && ign == 1){
      return mode_dev = 1;
    }else if(speed_kmh < 3   && ign == 1){
      return mode_dev = 2;
    }else if(speed_kmh > 100 && ign == 1){
      return mode_dev = 3;
    }
  return mode_dev;
}
String getModemDateTime() {
  modem.sendAT("+CCLK?");
  String datetime;  
  // Espera por la respuesta del comando AT
  if (modem.waitResponse(1000L, datetime) == 1) {
    // Buscar la posición de la primera comilla doble (donde empieza la fecha y hora)
    int start = datetime.indexOf("\"") + 1;
    int end = datetime.indexOf("\"", start);

    // Extraer la parte de la fecha y la hora
    String datetimeRaw = datetime.substring(start, end);
    // Extraer año, mes, día, hora, minuto y segundo del formato yy/MM/dd,hh:mm:ss
    String year = "20" + datetimeRaw.substring(0, 2); // Se asume que el año es 20XX
    String month = datetimeRaw.substring(3, 5);
    String day = datetimeRaw.substring(6, 8);
    String time = datetimeRaw.substring(9, 17); // hh:mm:ss
    
    // Formatear la salida final
    datetimeModem = year + month + day + ";" + time;
    return datetimeModem;
  } else {
    return "No date/time available.";
  }
}
String getDateTimeGPS(TinyGPSDate &d, TinyGPSTime &t) {
  String fecha = String(d.year()) + (d.month() < 10 ? "0" : "") + String(d.month()) + (d.day() < 10 ? "0" : "") + String(d.day());
  String hora = (t.hour() < 10 ? "0" : "") + String(t.hour()) + ":" + (t.minute() < 10 ? "0" : "") + String(t.minute()) + ":" + (t.second() < 10 ? "0" : "") + String(t.second());
  return fecha + ";" + hora;
}
int event_generated(int event){
  //AGREGAR VALIDACION GNSS PARA MOSTRAR HORA DE SIM CUANDO NO HAY FIX
  String data_event = "";
  
  Serial.print("EVENT => ");
  if(gnss_valid){
    data_event = String(headers[1])+";"+overwritten_imei+";3FFFFF;52;1.0.45;0;"+dateTime+";0000B0E2;334;20;1223;11;+"+String(latitude, 6)+";"+String(longitude, 6)+";"+speed_kmh+";"+course+";"+satellites+";"+fix+";00000"+in2+in1+ign+";000000"+out2+out1+";"+event+";;";
    }else{
      getModemDateTime();
      data_event = String(headers[1])+";"+overwritten_imei+";3FFFFF;52;1.0.45;0;"+datetimeModem+";0000B0E2;334;20;1223;11;+"+String(latitude, 6)+";-"+String(longitude, 6)+";"+speed_kmh+";"+course+";"+satellites+";"+fix+";00000"+in2+in1+ign+";000000"+out2+out1+";"+event+";;";
    }
    Serial.println(data_event);
    sendDATA(data_event); //si la respuesta es exitosa retorna event!
    delay(1000);
    return event;
}