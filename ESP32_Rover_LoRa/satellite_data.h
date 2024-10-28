/** RTK Data Page
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */
 
 const char sat_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Sensor Data Table</title>
  <style>
    table {
      border-collapse: collapse;
      width: 100%;
    }
    th, td {
      border: 1px solid black;
      padding: 8px;
      text-align: left;
    }
    th {
      background-color: #f2f2f2;
    }
  </style>
</head>
<body>
  <h1>Sattelite Data Table</h1>
  <table id="sat_table">
    <tr>
      <th>Constellation</th>
      <th>PRN</th>
      <th>Az</th>
      <th>El</th>
      <th>SNR</th>
    </tr>
  </table>

  <script>

    // Function to generate the table rows
    function generateTable() 
    {

        var newRow = sat_table.insertRow(sat_table.length);
        var cell1 = newRow.insertCell(0); cell1.innerHTML = sat_table_string[0];
        var cell2 = newRow.insertCell(1); cell2.innerHTML = sat_table_string[1];
        var cell3 = newRow.insertCell(2); cell3.innerHTML = sat_table_string[2];
        var cell4 = newRow.insertCell(3); cell4.innerHTML = sat_table_string[3];
        var cell5 = newRow.insertCell(4); cell5.innerHTML = sat_table_string[4];
        

//        for(auto it = gps_sat_map.begin(); it != gps_sat_map.end(); it++)
//        {
//           // Create a new row
//           var new_row = sat_table.insertRow(sat_table.length);
//           Serial.print("i");
//        
//           // Add values for cells in row
//           var col_1 = new_row.insertCell(0); col_1.innerHTML = it->second.constellation;
//           var col_2 = new_row.insertCell(1); col_2.innerHTML = it->first;
//           var col_3 = new_row.insertCell(2); col_3.innerHTML = it->second.azimuth;
//           var col_4 = new_row.insertCell(3); col_4.innerHTML = it->second.elevation;
//           var col_5 = new_row.insertCell(4); col_5.innerHTML = it->second.snr;
//        }
//
//        for(auto it = gal_sat_map.begin(); it != gal_sat_map.end(); it++)
//        {
//           // Create a new row
//           var new_row = sat_table.insertRow(sat_table.length);
//        
//           // Add values for cells in row
//           var col_1 = new_row.insertCell(0); col_1.innerHTML = it->second.constellation;
//           var col_2 = new_row.insertCell(1); col_2.innerHTML = it->first;
//           var col_3 = new_row.insertCell(2); col_3.innerHTML = it->second.azimuth;
//           var col_4 = new_row.insertCell(3); col_4.innerHTML = it->second.elevation;
//           var col_5 = new_row.insertCell(4); col_5.innerHTML = it->second.snr;
//        }
//        
//        for(auto it = bei_sat_map.begin(); it != bei_sat_map.end(); it++)
//        {
//           // Create a new row
//           var new_row = sat_table.insertRow(sat_table.length);
//        
//           // Add values for cells in row
//           var col_1 = new_row.insertCell(0); col_1.innerHTML = it->second.constellation;
//           var col_2 = new_row.insertCell(1); col_2.innerHTML = it->first;
//           var col_3 = new_row.insertCell(2); col_3.innerHTML = it->second.azimuth;
//           var col_4 = new_row.insertCell(3); col_4.innerHTML = it->second.elevation;
//           var col_5 = new_row.insertCell(4); col_5.innerHTML = it->second.snr;
//        }
    }

    // Call the function to generate the table on page load
    window.onload = generateTable;
  </script>
</body>
</html>

    )rawliteral";
