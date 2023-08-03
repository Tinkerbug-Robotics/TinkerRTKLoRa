/** GNSS Data Page
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */
const char gnss_html[] PROGMEM = R"rawliteral(
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
    <h1>TinkerRTK GNSS Data</h1>
    <p><a href='/'> Home</a></p>
  </div>
  <div class="content">
    <div class="cards">
      <div class="card">
        <p><i class="fas fa-globe" style="color:#FFA533;"></i> LATTITUDE</p><p><span class="reading"><span id="lat">%LATTITUDE%</span> &deg;</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-globe" style="color:#1EC80D;"></i> LONGITUDE</p><p><span class="reading"><span id="lng">%LONGITUDE%</span> &deg;</span></p>
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
 
 source.addEventListener('lattitude', function(e) {
  console.log("lattitude", e.data);
  document.getElementById("lat").innerHTML = e.data;
 }, false);
 
  source.addEventListener('longitude', function(e) {
  console.log("longitude", e.data);
  document.getElementById("lng").innerHTML = e.data;
 }, false);
 
}
</script>
</body>
</html>)rawliteral";
