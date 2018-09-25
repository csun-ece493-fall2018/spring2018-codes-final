/////////////////////////////////////////////
//   Include libraries
////////////////////////////////////////////
// Include the serialport library
const SerialPort = require('serialport');
// Include string_decoder library
const {StringDecoder} = require('string_decoder');
// Include query string library
const querystring = require('querystring');
// Include http library
const http = require('http');
// Define the decoding mode to be utf8
const decoder = new StringDecoder('utf8');
// Include websocket library
const WebSocket = require('ws');

const host = 'localhost:8888';

const rp = require('request-promise');
/////////////////////////////////////////////
//   Connect to the local serial port
////////////////////////////////////////////

var msg = "";
// Define the serial port to listen to
var port = new SerialPort("/dev/cu.usbmodem1421", {
  baudRate: 19200
}, function (err) {
  if (err) {
    return console.log('Error: ', err.message);
  }
});


port.on("open", function () {

    var answer = "90;";
    port.write(answer, function(err, results) {
                console.log("Port is open. Setting initial sensor angle to " + answer);
      }); 
});

/////////////////////////////////////////////
//   Connect to the remote server via websocket
////////////////////////////////////////////
//https://github.com/websockets/ws
const ws = new WebSocket('ws://' + host+'/ws');

ws.on('open', function open() {
    //ws.send('something');
    console.log("Connection established");
});

ws.on('message', function incoming(data) {
  var obj = JSON.parse(data);
  //console.log("Incomming id: ", obj["id"]);
  if (obj["id"] == 1 && obj["value"] == "r"){
    var request_val = "r~"; 
    
    port.write(request_val, function(err, results) {
            console.log("Incoming request : run the center servo",request_val );
      });
  }else if (obj["id"] == 1 && obj["value"] == "s"){
    var request_val = "s~"; 
    
    port.write(request_val, function(err, results) {
              console.log("Incoming request : stop the center servo", request_val);
      });
  }else if (obj["id"] == 2 && obj["value"] >= 15 && obj["value"] <= 90 ){
    var request_val = "a"+obj["value"]+"~"; 
    port.write(request_val, function(err, results) {
                console.log("Incoming request : position sensor servos to angle "+ request_val);
      });
  }else {
      // Do nothing
  }
});

/////////////////////////////////////////////
//   Read the data from the serial port
////////////////////////////////////////////

// Switches the port into "flowing mode"
port.on('data', function (data) {
  data_decoded = Buffer.from(data);
  if(data_decoded.indexOf('~') > -1 ){
    msg = msg+data_decoded;
    console.log(msg);
    var json_str = msg.split(',');
    if( json_str.length == 5){
    var run_center_servo = SanitizeUrl(json_str[0].replace(/~\s*$/, ""));
    var left_distance = SanitizeUrl(json_str[1].replace(/~\s*$/, "")); 
    var right_distance = SanitizeUrl(json_str[2].replace(/~\s*$/, "")); 
    var center_servo_angle = SanitizeUrl(json_str[3].replace(/~\s*$/, "")); 
    var sensor_servo_angle =  SanitizeUrl(json_str[4].replace(/~\s*$/, "")); 
    var to_be_send = "{\"run_center_servo\":\""+run_center_servo
    +"\",\"left_distance\":\""+left_distance
    +"\",\"right_distance\":\""+right_distance
    +"\",\"center_servo_angle\":\""+center_servo_angle
    +"\",\"sensor_servo_angle\":\""+sensor_servo_angle
    +"\",\"angleX\":\""+0
    +"\",\"angleY\":\""+0
    +"\",\"angleZ\":\""+0
    +"\",\"headingFiltered\":\""+0
    +"\"}~"; 
    //console.log(to_be_send);
    send_http_request("?id=3&value="+to_be_send);
  }
  msg = "";
  }else {
  msg = msg+data_decoded;
  }
});

/////////////////////////////////////////////
//   Send data to the server as Http request
////////////////////////////////////////////
function send_http_request(data) {
  console.log(data);
var options = {
  host: 'localhost', // In the production we will change the localhost to a public domain over the internet
  port: '8888',
  path: '/api'+data
};


var options = {
    uri: 'http://localhost:8888/api'+data,
    simple: false 
};
 

rp(options)
    .then(function (body) {
        // Process html like you would with jQuery...
    })
    .catch(function (err) {
        // Crawling failed or Cheerio choked...
        console.log("E--->",data,err);
    });
/*
var req = http.get(options, function(res) {
  //console.log('STATUS: ' + res.statusCode);
  //console.log('HEADERS: ' + JSON.stringify(res.headers));

  // Buffer the body entirely for processing as a whole.
  var bodyChunks = [];
  res.on('data', function(chunk) {
    // You can process streamed parts here...
    bodyChunks.push(chunk);
  }).on('end', function() {
    var body = Buffer.concat(bodyChunks);
    //console.log('BODY: ' + body);
    // ...and/or process the entire body here.
  }).on('error', function(err) {
    //console.log(err);
  })
});
*/
}

function SanitizeUrl(str)
{
    newStr = str.replace("[^-A-Za-z0-9+&@#/%?=~_|!:,.;\(\)]", "");
    return newStr;
}