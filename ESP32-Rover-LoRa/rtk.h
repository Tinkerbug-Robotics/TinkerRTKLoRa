/** RTK Data Page
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */
const char rtk_html[] PROGMEM = R"rawliteral(
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
    <h1>TinkerRTK RTK Data</h1>
    <p><a href='/'> Home</a></p>
  </div>
  
  <div class="content">
    <div class="cards">

      <div class="card">
        <p><i class="fa-solid fa-truck-fast" style="color:#FFA533;"></i> MODE</p><p><span class="reading"><span id="rtk_mode">%RTK_MODE%</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-clock" style="color:#0B67EC;"></i> DATE</p><p><span class="reading"><span id="gnss_date">%GNSS_DATE%</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-clock" style="color:#e1e437;"></i> UTC TIME</p><p><span class="reading"><span id="gnss_time">%GNSS_TIME%</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-clock" style="color:#1EC80D;"></i> UP TIME</p><p><span class="reading"><span id="up_time">%UP_TIME%</span> minutes</span></p>
      </div>
      <div class="card">
        <p><i class="fa-solid fa-satellite-dish" style="color:#0B67EC;"></i> FIX TYPE</p><p><span class="reading"><span id="fix">%FIX%</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-calendar" style="color:#0B67EC;"></i> RTK AGE</p><p><span class="reading"><span id="rtk_age">%RTK_AGE%</span> sec</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-check" style="color:#0B67EC;"></i> RTK Ratio</p><p><span class="reading"><span id="rtk_ratio">%RTK_RATIO%</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-satellite" style="color:#0B67EC;"></i> CYCLE SLIPS</p><p><span class="reading"><span id="cs_gps">%CS_GPS%</span> / <span id="cs_bds">%CS_BDS%</span> / <span id="cs_gal">%CS_GAL%</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-compass" style="color:#FFA533;"></i> EAST</p><p><span class="reading"><span id="rtk_east">%RTK_EAST%</span> m</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-compass" style="color:#FFA533;"></i> NORTH</p><p><span class="reading"><span id="rtk_north">%RTK_NORTH%</span> m</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-compass" style="color:#FFA533;"></i> UP</p><p><span class="reading"><span id="rtk_up">%RTK_UP%</span> m</span></p>
      </div>
    </div>
  </div>
<script>
if (!!window.EventSource) 
{
 var source = new EventSource('/events');
 
 source.addEventListener('open', function(e) 
 {
  console.log("Events Connected");
 }, false);
 
 source.addEventListener('error', function(e) 
 {
  if (e.target.readyState != EventSource.OPEN) 
  {
    console.log("Events Disconnected");
  }
 }, false);

  source.addEventListener('rtk_mode', function(e) 
  {
    console.log("rtk_mode", e.data);
    document.getElementById("rtk_mode").innerHTML = e.data;
  }, false);
  
  source.addEventListener('gnss_date', function(e) 
  {
    console.log("gnss_date", e.data);
    document.getElementById("gnss_date").innerHTML = e.data;
  }, false);
  
  source.addEventListener('gnss_time', function(e) 
  {
    console.log("gnss_time", e.data);
    document.getElementById("gnss_time").innerHTML = e.data;
  }, false);
  
  source.addEventListener('up_time', function(e) 
  {
    console.log("up_time", e.data);
    document.getElementById("up_time").innerHTML = e.data;
  }, false);

  source.addEventListener('fix', function(e) 
  {
    console.log("fix", e.data);
    document.getElementById("fix").innerHTML = e.data;
  }, false);

  source.addEventListener('rtk_age', function(e) 
  {
    console.log("rtk_age", e.data);
    document.getElementById("rtk_age").innerHTML = e.data;
  }, false);
  
  source.addEventListener('rtk_ratio', function(e) 
  {
    console.log("rtk_ratio", e.data);
    document.getElementById("rtk_ratio").innerHTML = e.data;
  }, false);
   
  source.addEventListener('cs_gps', function(e) 
  {
    console.log("cs_gps", e.data);
    document.getElementById("cs_gps").innerHTML = e.data;
  }, false);

  source.addEventListener('cs_bds', function(e) 
  {
    console.log("cs_bds", e.data);
    document.getElementById("cs_bds").innerHTML = e.data;
  }, false);
  
  source.addEventListener('cs_gal', function(e) 
  {
    console.log("cs_gal", e.data);
    document.getElementById("cs_gal").innerHTML = e.data;
  }, false);

  source.addEventListener('rtk_east', function(e) 
  {
    console.log("rtk_east", e.data);
    document.getElementById("rtk_east").innerHTML = e.data;
  }, false);
  
  source.addEventListener('rtk_north', function(e) 
  {
    console.log("rtk_north", e.data);
    document.getElementById("rtk_north").innerHTML = e.data;
  }, false);
  
  source.addEventListener('rtk_up', function(e) 
  {
    console.log("rtk_up", e.data);
    document.getElementById("rtk_up").innerHTML = e.data;
  }, false);
}
</script>
</body>
</html>)rawliteral";
