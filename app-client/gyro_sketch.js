angleX = 0;
angleY = 0;
angleZ = 0;
rotx = 0;
roty = 0;
var host = 'localhost:8888';
var socket; // the websocket

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

    //push();
    noFill();
    stroke(255);
    //rotateX(rotx);
    //rotateY(roty);
    box(250, 250, 250);

    noFill();
    stroke(98,245,31);
    //rotateX(angleX);
    //rotateY(angleY);
    //rotateZ(angleZ);
    rotateX(rotx);
    rotateY(roty);
    //rotateZ(45);
    
    ellipse(0, 0, 200);
    ellipse(0, 0, 150);
    ellipse(0, 0, 100);
    //pop();
    //rotateZ(frameCount * 0.01)

}

function mouseDragged() {
    rate = 0.01;
    rotx += (pmouseY-mouseY) * rate;
    roty += (mouseX-pmouseX) * rate;
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
    
    if(json_data["angleX"]){
      angleX = Number(json_data["angleX"]);
    }
    if(json_data["angleY"]){
      angleY = Number(json_data["angleY"]);
    }
    if(json_data["angleZ"]){
      angleZ = Number(json_data["angleZ"]);
    }
  console.log(angleX,angleY,angleZ);
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