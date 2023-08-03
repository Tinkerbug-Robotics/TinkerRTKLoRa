/** RTK Data Page
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */
 
 const char rtk_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>TinkerRTK</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
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
        <p><i class="fas fa-clock" style="color:#0B67EC;"></i> UP TIME</p><p><span class="reading"><span id="up_time">%UP_TIME%</span> minutes</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-clock" style="color:#0B67EC;"></i> UPLOADS</p><p><span class="reading"><span id="num_uploads">%NUM_UPLOADS%</span></p>
      </div>
    </div>
  </div>
<script>
if (!!window.EventSource) {
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
 
  source.addEventListener('up_time', function(e) 
  {
    console.log("up_time", e.data);
    document.getElementById("up_time").innerHTML = e.data;
  }, false);

  source.addEventListener('num_uploads', function(e) 
  {
    console.log("num_uploads", e.data);
    document.getElementById("num_uploads").innerHTML = e.data;
  }, false);
 
}
</script>
</body>
</html>)rawliteral";
