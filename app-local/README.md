# Local Application
## Local NodeJs server to listen to a serial port on the local machine and send the read data to an external server

To start make sure to have all the dependencies installed on your virtual environment.

```javascript
npm install --save serialport
```
https://www.npmjs.com/package/serialport

```javascript
npm install --save string_decoder
```
https://www.npmjs.com/package/string_decoder

```javascript
npm install --save querystring
```
https://github.com/Gozala/querystring

```javscript
npm install --save http
```

The following code will create a serial prot listener

```javascript
var port = new SerialPort("/dev/cu.usbmodem1421", {
  baudRate: 9600
}, function (err) {
  if (err) {
    return console.log('Error: ', err.message);
  }
});
```
The following code will read the serial port as buffer and decode them as string.
It will concatinate all the characters up to ~ sign and will send them to http_request function

```javascript
port.on('data', function (data) {
  data_decoded = Buffer.from(data);
  if(data_decoded.indexOf('~') > -1 ){
    msg = msg+data_decoded;
    console.log(msg);
    send_http_request("?id=1&value="+msg);
  msg = "";
  }else {
  msg = msg+data_decoded;
  }
});
```

The following code will send the data as GET request in a json format

```javascript
var req = http.get(options, function(res) {
  console.log('STATUS: ' + res.statusCode);
  console.log('HEADERS: ' + JSON.stringify(res.headers));

  // Buffer the body entirely for processing as a whole.
  var bodyChunks = [];
  res.on('data', function(chunk) {
    // You can process streamed parts here...
    bodyChunks.push(chunk);
  }).on('end', function() {
    var body = Buffer.concat(bodyChunks);
    console.log('BODY: ' + body);
    // ...and/or process the entire body here.
  })
});

}
```