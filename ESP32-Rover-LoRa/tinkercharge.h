/** TinkerCharge Data Page
 *  Displays values from TinkerCharge board sent from RP2040.
 *  Information relates to the power usage, voltage, and state of charge of the battery.
 *  Copyright Tinkerbug Robotics 2023
 *  Provided under GNU GPL 3.0 License
 */
 
 const char tc_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>TinkerRTK</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    p { font-size: 1.2rem;}
    body {  margin: 0;}
    .topnav { overflow: hidden; background-color: #BF17F9; color: white; font-size: 1rem; }
    .content { padding: 20px; }
    .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }
    .cards { max-width: 800px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); }
    .reading { font-size: 1.4rem; }
  </style>
</head>
<body>
  <div class="topnav">
    <h1>TinkerRTK TinkerCharge Data</h1>
    <p><a href='/'> Home</a></p>
    
    
  </div>
  <div class="content">
    <div class="cards">
      <div class="card">
        <p><i class="fas fa-bolt" style="color:#FFA533;"></i> VOLTAGE</p><p><span class="reading"><span id="volt">%VOLTAGE%</span> V</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-bolt" style="color:#e1e437;"></i> AVG VOLTAGE</p><p><span class="reading"><span id="a_volt">%AVG_VOLTAGE%</span> V</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-arrow-right" style="color:#FFA533;"></i> CURRENT</p><p><span class="reading"><span id="curr">%CURRENT%</span> mA</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-arrow-right" style="color:#e1e437;"></i> AVG CURRENT</p><p><span class="reading"><span id="a_curr">%AVG_CURRENT%</span> mA</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-battery-full" style="color:#0B67EC;"></i> BATTERY CHARGE</p><p><span class="reading"><span id="b_soc">%BATT_CHARGE%</span> &percnt;</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-battery-full" style="color:#1EC80D;"></i> BATTERY CAPACITY</p><p><span class="reading"><span id="b_cap">%BATT_CAPACITY%</span> mAH</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-thermometer-half" style="color:#0B67EC;"></i> TC TEMPERATURE</p><p><span class="reading"><span id="temp">%TEMPERATURE%</span> &deg;C</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-clock" style="color:#0B67EC;"></i> UP TIME</p><p><span class="reading"><span id="up_time">%UP_TIME%</span> minutes</span></p>
      </div>
    </div>
  </div>
<script>
if (!!window.EventSource) {
 var source = new EventSource('/events');
 
 source.addEventListener('open', function(e) {
  console.log("Events Connected");
 }, false);
 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");
  }
 }, false);
 
 source.addEventListener('message', function(e) {
  console.log("message", e.data);
 }, false);
 
 source.addEventListener('voltage', function(e) {
  console.log("voltage", e.data);
  document.getElementById("volt").innerHTML = e.data;
 }, false);
 
  source.addEventListener('avg_voltage', function(e) {
  console.log("avg_voltage", e.data);
  document.getElementById("a_volt").innerHTML = e.data;
 }, false);
 
 source.addEventListener('current', function(e) {
  console.log("current", e.data);
  document.getElementById("curr").innerHTML = e.data;
 }, false);

  source.addEventListener('avg_current', function(e) {
  console.log("avg_current", e.data);
  document.getElementById("a_curr").innerHTML = e.data;
 }, false);

   source.addEventListener('battery_capacity', function(e) {
  console.log("battery_capacity", e.data);
  document.getElementById("b_cap").innerHTML = e.data;
 }, false);

   source.addEventListener('battery_soc', function(e) {
  console.log("battery_soc", e.data);
  document.getElementById("b_soc").innerHTML = e.data;
 }, false);

    source.addEventListener('tc_temp', function(e) {
  console.log("tc_temp", e.data);
  document.getElementById("temp").innerHTML = e.data;
 }, false);

     source.addEventListener('up_time', function(e) {
  console.log("up_time", e.data);
  document.getElementById("up_time").innerHTML = e.data;
 }, false);
 
}
</script>
</body>
</html>)rawliteral";
