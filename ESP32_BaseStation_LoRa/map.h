/**********************************************************************
*
* Copyright (c) 2023 Tinkerbug Robotics
*
* This program is free software: you can redistribute it and/or modify it under the terms
* of the GNU General Public License as published by the Free Software Foundation, either
* version 3 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
* PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this 
* program. If not, see <https://www.gnu.org/licenses/>.
* 
* Authors: 
* Christian Pedersen; tinkerbug@tinkerbugrobotics.com
* 
**********************************************************************/
 
 const char map_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>TinkerRTK GNSS Map </title>
  <link rel = "stylesheet" href = "http://cdn.leafletjs.com/leaflet-0.7.3/leaflet.css"/>
</head>
<body>
   <h1>TinkerRTK GNSS Map</h1>
   <p><a href='/'> Home</a></p>
   <div id="map" style = "width: 900px; height: 580px"></div>
   <script src = "http://cdn.leafletjs.com/leaflet-0.7.3/leaflet.js"></script>
<script>
   var mapOptions = 
   {
     center: [39.28150740469, -74.5583502359],
     zoom: 10
  };
  
  var map = new L.map('map', mapOptions);

  var layer = new L.TileLayer("http://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png");

  map.addLayer(layer);

  var latlng = L.latLng(0.0, 0.0);
  var marker = L.marker(latlng).addTo(map);

  function getPosition()
  {
    xmlhttp=new XMLHttpRequest();
    xmlhttp.open('GET','/loc',false);
    xmlhttp.send();

    var data = xmlhttp.responseText;
    console.log(data);
    vals = data.split(',');
    latlng = L.latLng(parseFloat(vals[0]), parseFloat(vals[1]));
    map.panTo(latlng);
    
    updateMarker();
  }

  function updateMarker() 
  {
     marker.setLatLng(latlng);
  }

  setInterval(getPosition,2000);
  
</script>
</body>
</html>)rawliteral";
