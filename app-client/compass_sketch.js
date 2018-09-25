var headingFiltered = 10;
var host = 'localhost:8888';
var socket; // the websocket

function preload(){
	imgCompass = loadImage("compass.png");
	imgCompassArrow = loadImage("compass_arrow.png");
}
function setup(){ 
  createCanvas(window.innerWidth, window.innerHeight, WEBGL);
  socket = new WebSocket('ws://' + host+'/ws');
  // socket connection listener:
  socket.onopen = sendIntro;
  // socket message listener:
  socket.onmessage = readMessage;


 }

function draw() {
  background(0);
  push();
  rotateZ(radians(-headingFiltered));
  
  strokeWeight(5);
  stroke(98,245,31);
  fill(0);
  ellipse(0, 0, 400);
  ellipse(0, 0, 300);
  line(0,-150,0,-220);
  pop();

  push();
  translate(0,+50);
  fill(255);
  noStroke();
  cone(25,100);
  pop();
  push();
  translate(0,-50);
  rotateZ(radians(180));
  fill(98,245,31);
  noStroke();
  cone(25,100);
  pop();
	
  

}


function sendIntro() {
  // convert the message object to a string and send it:
  //socket.send("Hello");
}

function readMessage(event) {

  var msg = JSON.parse(event.data);
  // console.log(event);
  if(msg["id"] == 3){
    serialEventHandler(msg.value);
  }
}


function serialEventHandler(myPort) { // starts reading data from the Serial Port


  var data = myPort;
  var last_char = data.substr(data.length - 1);

  if(last_char === '~'){
    data = data.slice(0,-1);
  }
  
  if(isJson(data)){
    json_data = JSON.parse(data);
    
    if(json_data["headingFiltered"]){
      headingFiltered = Number(json_data["headingFiltered"]);
    }
   
  console.log(headingFiltered);
  }

}

function isJson(str) {
    try {
        JSON.parse(str);
    } catch (e) {
        return false;
    }
    return true;
}