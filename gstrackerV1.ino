/*

NOTA: Es necesario validar la posición(fix) todo el tiempo.
1. si no tiene posición valida manda el ultimo lat y lon con fix.
2. manda la hora de la sim cuando no haya posición valida
3. al ya tener posición valida actualiza las variables lat y lon.
4. crear una variable tipo buffer para guardar el ultimo trackeo mandado al servidor
5. si no tiene fix manda las coordenas del ultimo buffer

*/
#include <Arduino.h>
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial serialgps(44,43);
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

const char *apns[] = {
    "ott.iot.attmex.mx",
    "internet.itelcel.com"
};
String inputCommand = ""; // Variable para almacenar el comando ingresado

String imei, ccid, imsi, cop, message, data, datetimeModem;
int  csq;
bool gnss_valid = false;
bool gprs_state;
float test_lat, test_lon;
float lat2 = 0.000000, lon2 = 0.000000;
float speed2    = 0;
float alt2      = 0;
int   vsat2     = 0;
int   usat2     = 0;
float accuracy2 = 0;
char year2[5]  = "0000";  // 4 dígitos + 1 para el terminador '\0'
char month2[3] = "00";    // 2 dígitos + 1 para el terminador '\0'
char day2[3]   = "00";    // 2 dígitos + 1 para el terminador '\0'
char hour2[3]  = "00";    // 2 dígitos + 1 para el terminador '\0'
char min2[3]   = "00";    // 2 dígitos + 1 para el terminador '\0'
char sec2[3]    = "00";    // 2 dígitos + 1 para el terminador '\0'
bool level     = false;
int  fix       = 0;
float  power_in    = 0.00;
float  backup_batt = 0.00;
bool powerOff = false;
bool powerOn = false;

/*const char* server = "201.99.112.78";//V4 DEV
const int port = 6100;*/

/*const char* server = "18.233.36.4"; //V3 PROD
const int port = 5208;*/

const char* server = "34.196.135.179"; //V3 PROD
const char* overwritten_imei = "6047417071";
const int port = 5200;
const char *headers[] = {
    "STT",
    "ALT",
    "RES",
    "CMD"
};
String readcommand = "";

bool last_valid_position;
char last_valid_lat2[10], last_valid_lon2[10];
//const int GPO1  = 1; //back up
const int GPO8  = 8; //Power in (sheld 1.1)
const int GPO9  = 9; //power in (back up sheld 1.1)
const int GPO10 = 46; // ign;rojo
const int GPO11 = 11;// in1;azul
const int GPO12 = 12; //in2
const int GPO13 = 13; //out1;amarillo
const int GPO14 = 14; //out2

const int R1_divisor_power_in = 4700;//we read the voltage acrosst this resistor (car resistors)
const int R2_divisor_power_in = 43000;
const int R1_divisor_batt = 5100;//we read the voltage acrosst this resistor (backup resistors)
const int R2_divisor_batt = 1800;

int ign = 0, mode = 0, in1 = 0, in2 = 0, out1 = 0, out2 = 0;
bool estadoPulsadorAnterior = HIGH;
bool LaststateIgnition = HIGH;
bool LaststateInput1 = HIGH;

unsigned long previousMillis = 0;   // Guarda el tiempo del último evento
const long interval = 7000;        // Intervalo de tiempo en milisegundos (10 segundos)

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static float get_float_value(float val, float invalid, int len, int prec);
static int get_int_value(unsigned long val, unsigned long invalid, int len);

void setup() {
  Serial.begin(115200);
   serialgps.begin(9600); 
  //Start while waiting for Serial monitoring
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

    //Modem GPS Power channel
  PMU.setBLDO2Voltage(3300);
  PMU.enableBLDO2();     
  pinMode(GPO13, OUTPUT);
  pinMode(GPO10,INPUT_PULLUP);
  pinMode(GPO11,INPUT_PULLUP);
  pinMode(GPO12,OUTPUT);
  digitalWrite(GPO12,HIGH);

  Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);
  configModem();
}

void loop() {
  // Config modem
  if(!gprs_state){
    //Serial.println("Config modém => ");
    configModem();
  }
  // get data GNSS
  //agregar la validacion otra vez del fix y position_valid
    float flat, flon;
    unsigned long age;

    gps.f_get_position(&flat, &flon, &age);
      // Almacena las coordenadas con 5 decimales
    lat2 = (float)flat;
    lon2 = (float)flon;

    // Consult data GPS module
    /*print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
    print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
    print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);*/
    //print_int(age, TinyGPS::GPS_INVALID_AGE, 5);

    lat2 = get_float_value(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    lon2 = get_float_value(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
    fix = get_int_value(age, TinyGPS::GPS_INVALID_AGE, 5);
    vsat2 = get_int_value(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
    //speed2 = get_float_value(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
    speed2 = gps.f_speed_kmph();
    print_date(gps);    
  
  // Inputs/Outputs state

  // Calculate data
  mode = mode_divice();
  power_in = get_power_value();
  backup_batt = get_batt_value();

  // Events emit
  ignition_event();
  input_event();
  panic_event();
  power_conn_event(power_in);
  // Consult data modem
  /*if(gprs_state){
    dataModem();
  }*/

  //Format data GNSS
  if(speed2 == -1.00 || speed2 == 0){
    speed2 = 0.00; 
  }
  //########################################################################
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  // Leer los comandos ingresados en el monitor serial
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    // Ignorar caracteres de nueva línea y retorno de carro
    if (inChar != '\n' && inChar != '\r') {
      inputCommand += inChar;
    }

    // Enviar el comando al módulo SIM cuando se presiona Enter
    if (inChar == '\n' || inChar == '\r') {
      if (inputCommand.length() > 0) {
        Serial1.println(inputCommand);
        inputCommand = ""; // Limpiar la variable después de enviar el comando
      }
    }
  }
  //###################################################################
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Si han pasado 10 segundos, ejecuta la función y actualiza el tiempo anterior
    previousMillis = currentMillis;
    Serial.print("DATA => ");
    if(gnss_valid){
        Serial.println("Posición válida!");
        data = String(headers[0])+";"+overwritten_imei+";3FFFFF;95;1.0.21;1;"+String(year2)+month2+day2+";"+hour2+":"+min2+":"+sec2+";04BB4A02;334;20;3C1F;18;+"+String(lat2, 6)+";"+String(lon2, 6)+";"+speed2+";81.36;"+3+";"+1+";00000"+in2+in1+ign+";000000"+out2+out1+";"+mode+";1;0929;"+backup_batt+";"+power_in;
    }else{
      Serial.println("SIN Posición válida!");
      getModemDateTime();
      data = String(headers[0])+";"+overwritten_imei+";3FFFFF;95;1.0.21;1;"+datetimeModem+";04BB4A02;334;20;3C1F;18;+"+lat2+"0000;-"+lon2+"0000;"+String(speed2)+";81.36;"+vsat2+";"+fix+";00000"+in2+in1+ign+";000000"+out2+out1+";"+mode+";1;0929;"+backup_batt+";"+power_in;
      //data  = "STT;6047417071;3FFFFF;95;1.0.21;1;20240923;08:39:16;04BB4A02;334;20;3C1F;18;+21.020603;-89.585097;0.19;81.36;17;1;00000001;00000000;1;1;0929;4.1;14.19";
    }
    Serial.println(data);
    sendDATA(data);
  }
  //EL TIEMPO DE RESPUESTA DE EVENTOS, TRACKEOS TODO LO QUE ESTÁ EN EL LOOP() TARDARÁ EL TIEMPO EN GENERARSE QUE ESTÉ ESTE DELAY()
  //delay(6000);
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
void dataModem(){
  ccid = modem.getSimCCID();
  Serial.print("CCID:");
  Serial.println(ccid);

  imei = modem.getIMEI();
  Serial.print("IMEI:");
  Serial.println(imei);

  imsi = modem.getIMSI();
  Serial.print("IMSI:");
  Serial.println(imsi);

  cop = modem.getOperator();
  Serial.print("Operator:");
  Serial.println(cop);

  IPAddress local = modem.localIP();
  Serial.print("Local IP:");
  Serial.println(local);

  csq = modem.getSignalQuality();
  Serial.print("Signal quality:");
  Serial.println(csq);
  
 
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
  String response = String(headers[2])+";"+overwritten_imei+";"+String(year2)+month2+day2+";"+hour2+":"+min2+":"+sec2+";000039C5;"+String(lat2, 6)+";"+String(lon2, 6)+";"+speed2+";81.36;"+vsat2+";"+fix+";2958851;"+power_in+";00000"+in2+in1+ign+";000000"+out2+out1+";0;0";
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
/*void readData(){
  modem.sendAT("+CARECV=0,100");
  if (modem.waitResponse() != 1) {
    Serial.println("Enable DATA TCPSERVER!");
  }
  Serial.print("READ DATA: ");
  Serial.println(modem.waitResponse()); 
}*/
static void smartdelay(unsigned long ms){ //solo para gps 
  unsigned long start = millis();
  do {
    while (serialgps.available())
      gps.encode(serialgps.read());
  } while (millis() - start < ms);
}
char* get_arrchar_value(float val, float invalid, int len, int prec) {
  static char buffer[32]; // Declara el buffer como estático para que no se destruya al salir de la función

  if (val == invalid) {
    strcpy(buffer, "00.00"); // Copia la cadena de error en el buffer
    //gnss_valid = false;
  } else {
    // Redondear el valor a la precisión deseada
    float factor = pow(10, prec);
    float rounded_val = round(val * factor) / factor;
    // Convertir el valor redondeado a cadena con la precisión deseada
    dtostrf(rounded_val, len, prec, buffer);
    // Eliminar espacios en blanco al inicio y final
    char* trimmed = buffer;
    while(*trimmed == ' '){
      trimmed++; // Desplaza el puntero para evitar espacios en blanco
    } 
    //gnss_valid = true;
  }
  return buffer; // Devuelve el buffer
}

static void print_float(float val, float invalid, int len, int prec) {
  if (val == invalid){
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }else{
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i){
      Serial.print(' ');
    }
  }
  smartdelay(0);
}
static void print_int(unsigned long val, unsigned long invalid, int len){
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}
static void print_date(TinyGPS &gps){
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE){
    //Serial.println("********** NO DATA GNSS ********** ");
  }else {
    sprintf(year2, "%04d", year);  // Convierte el entero a una cadena con formato de 4 dígitos
    sprintf(month2, "%02d", month);
    sprintf(day2, "%02d", day);  // Convierte el entero a una cadena con formato de 4 dígitos
    sprintf(hour2, "%02d", hour);
    sprintf(min2, "%02d", minute);
    sprintf(sec2, "%02d", second);
    
    char sz[32];
    //sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ", month, day, year, hour, minute, second);
   // Serial.println(sz);
  }
  //print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
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
int get_int_value(unsigned long val, unsigned long invalid, int len) {
  char sz[32];  // Cadena para almacenar el número formateado
  if (val == invalid) {
        //gnss_valid = false;
    strcpy(sz, "**** NO DATA INT ***");  // En caso de valor inválido
  } else {
    sprintf(sz, "%ld", val);  // Convertir el valor en una cadena
  }
  sz[len] = 0;  // Asegurarse de que la cadena termine correctamente
  for (int i = strlen(sz); i < len; ++i) {
    sz[i] = ' ';  // Ajustar el largo de la cadena
  }
  if (len > 0) {
    sz[len - 1] = ' ';  // Colocar un espacio al final de la cadena
  }
  // Convertir la cadena a un valor entero y retornarlo
  int intValue = atoi(sz);
  return intValue;
}
float get_float_value(float val, float invalid, int len, int prec) {
  if (val == invalid) {
    gnss_valid = false;
    return 00.0;  // Devuelve un valor especial si el valor es inválido
  } else {
    // Redondear el valor a la precisión deseada
    float factor = pow(10, prec);
    float rounded_val = round(val * factor) / factor;
    // Calcular la longitud del número para agregar espacios si es necesario
    int vi = abs((int)rounded_val);
    int flen = prec + (rounded_val < 0.0 ? 2 : 1);  // Cuenta '.' y '-'
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;

    // Imprimir espacios adicionales para cumplir con la longitud 'len'
    for (int i = flen; i < len; ++i) {
      Serial.print(' ');
    }
    // Retornar el valor flotante redondeado
    gnss_valid = true;
    return rounded_val;
  }
}
void ignition_event(){
  int StateIgnition = digitalRead(GPO10);
  if (StateIgnition == LOW && LaststateIgnition == HIGH) {
    Serial.println("********* ¡ignition ON! ********** ");
    ign = 1;
     event_generated(IGN_ON);
  }else if(StateIgnition == HIGH && LaststateIgnition == LOW){
    Serial.println("********** ¡ignition OFF! *********** ");
    ign = 0;
    event_generated(IGN_OFF);
  }
  LaststateIgnition = StateIgnition;
  delay(200);
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
  delay(200); 
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
    if(speed2 > 10  && ign == 0){
      return mode_dev = 0;
    }else if(speed2 > 10  && ign == 1){
      return mode_dev = 1;
    }else if(speed2 < 3   && ign == 1){
      return mode_dev = 2;
    }else if(speed2 > 100 && ign == 1){
      return mode_dev = 3;
    }
  return mode_dev;
}
int event_generated(int event){
  //AGREGAR VALIDACION GNSS PARA MOSTRAR HORA DE SIM CUANDO NO HAY FIX
  String data_event = "";
  
  Serial.print("EVENT => ");
  if(gnss_valid){
    data_event = String(headers[1])+";"+overwritten_imei+";3FFFFF;52;1.0.45;0;"+String(year2)+month2+day2+";"+hour2+":"+min2+":"+sec2+";0000B0E2;334;20;1223;11;+"+String(lat2, 6)+";"+String(lon2, 6)+";"+speed2+";81.36;"+vsat2+";"+1+";00000"+in2+in1+ign+";000000"+out2+out1+";"+event+";;";
    }else{
    float speed_err = (speed2 == -1.00) ? 0.00 : speed2;
      getModemDateTime();
      data_event = String(headers[1])+";"+overwritten_imei+";3FFFFF;52;1.0.45;0;"+datetimeModem+";0000B0E2;334;20;1223;11;+"+String(lat2, 6)+";-"+String(lon2, 6)+";"+speed_err+";81.36;"+vsat2+";"+fix+";00000"+in2+in1+ign+";000000"+out2+out1+";"+event+";;";
    }
    Serial.println(data_event);
    sendDATA(data_event); //si la respuesta es exitosa retorna event!
    delay(1000);
    return event;
}