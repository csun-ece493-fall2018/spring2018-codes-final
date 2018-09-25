var grid_steps = 10;
var max_distance = 100;
var number_of_circles = grid_steps/2;
var distance_between_sensor_1_2 = 70;
var left_distance = 0;
var right_distance= 0;
var radius = 0;

var left_sensor_x1 = 0;
var left_sensor_y1 = 0;
var left_sensor_x2 = 0;
var left_sensor_y2 = 0;

var right_sensor_x1 = 0;
var right_sensor_y1 = 0;
var right_sensor_x2 = 0;
var right_sensor_y2 = 0;

var object_x = 0;
var object_y = 0;

var x = 0;
var y = 0;
var r = 0;
var thera = 0;

var sensor_angle = 90;
var current_angle = 90;

var center_servo_current_angle = 90;
var center_servo_status = 0;

var sensor_angle_coverage = 30;

var show_sensor_coverage = false;

var sensor_max_distance = 0;
var sensor_min_distance = 0;

var host = 'localhost:8888';
var socket; // the websocket

var start_reading = false;
var window_fill = 50;

var show_object_lines = false;
function setup(){ 
  createCanvas(window.innerWidth, window.innerHeight);
  pixelDensity(1);
  smooth();
  draw_control_box(window.innerWidth, window.innerHeight);
  socket = new WebSocket('ws://' + host+'/ws');
  // socket connection listener:
  socket.onopen = sendIntro;
  // socket message listener:
  socket.onmessage = readMessage;
 }

 function draw(){
  radius = get_radius();
 	draw_grid(radius);
  draw_grid_text(radius);
  draw_sensor_stick(radius);
  //draw_circle(grid_steps/2, radius);

  var px_distance_between_sensor_1_2 = Number(distance_between_sensor_1_2)*2*radius/max_distance;

  if(show_object_lines){
  push();
  translate((width/2)-(px_distance_between_sensor_1_2/2),(height/2)+radius);
  strokeWeight(2);
  stroke(253,245,99);
  line(left_sensor_x1,left_sensor_y1,left_sensor_x2,left_sensor_y2);
  strokeWeight(15);
  line(0,0,0,0);
  pop();

  push();
  translate((width/2)+(px_distance_between_sensor_1_2/2),(height/2)+radius);
  strokeWeight(2);
  stroke(253,245,99);
  line(right_sensor_x1,right_sensor_y1,right_sensor_x2,right_sensor_y2);
  strokeWeight(15);
  line(0,0,0,0);
  pop();
  }
  push();
  if(object_x != 0 && object_y != 0){
    translate((width/2)-(px_distance_between_sensor_1_2/2),(height/2)+radius);
    ellipse(object_x, object_y, 20, 20);
    if(show_object_lines){
    stroke(255,255,255);    
    strokeWeight(3);
    line(0+(px_distance_between_sensor_1_2/2),0,object_x, object_y);
  }
  }
  pop();
  

  push();
  translate((width/2),(height/2)+radius);
  strokeWeight(15);
  stroke(255,255,255);
  line(0,0,0,0);
  pop();

  sensor_angle_coverage = slider_1.value();
  label8.html('Sensor coverage ('+sensor_angle_coverage+'°)');

  sensor_angle = slider_2.value();
  label9.html('Sensor angle ('+sensor_angle+'°)');
  
  draw_sensor_coverage(radius);
  

  label_r_10.html('max: '+Math.round(sensor_max_distance));
  label_r_11.html('min: '+Math.round(sensor_min_distance));

  if(start_reading){
    show_on_2d_plane();
  }
  draw_window();
 }

 function draw_window(){
  push();
  // simulating motion blur and slow fade of the moving line
  fill(98,245,31);
  noStroke();
  fill(0,window_fill); 
  //background(0,2);
  rect(0, 0, width, height);
  pop();
}

function draw_grid(radius){
  push();
  translate(width/2,height/2);
	var x1 = -radius;
	var x2 = radius;
	var y1 = -radius;
	var y2 = radius;

  var horizontal_diff = abs(x2 - x1);
  var horizontal_steps = horizontal_diff/grid_steps;

  var vertial_diff = abs(y2 - y1);
	var vertiacal_steps = vertial_diff/grid_steps;

	stroke(98,245,31);
  var counter1 = 0;
	var i = 0;
  
  for(i = y1; i <= y2; i = i + vertiacal_steps){

		line(x1,i,x2,i);
    counter1++;
	}
  if (counter1 < grid_steps+1){
    line(x1,i,x2,i);
  }

  var counter2 = 0;
  var j = 0;

	for(j = x1; j <= x2; j = j + horizontal_steps){
		line(j,y1,j,y2);
	  counter2++;
  }
  if (counter2 < (grid_steps+1)){
    line(j,y1,j,y2);
  }
	pop();

}

function draw_circle(number_of_circles, radius) {
  push();
  noFill();
  translate(width/2,height/2);
  strokeWeight(2);
  stroke(98,245,31); // green color
  ellipseMode(RADIUS);  // Set ellipseMode to RADIUS
  var circle_step = 100 / number_of_circles;
  var temp_circle_step = 0.00;
  ellipse(0, 0, radius*0.001, radius*0.001);
  for (var i=0; i < number_of_circles; i++){
    temp_circle_step = temp_circle_step+circle_step;
    ellipse(0, 0, radius*(temp_circle_step/100), radius*(temp_circle_step/100));
  }
  pop();
}

function draw_grid_text(radius){
  var x1 = -radius;
  var x2 = radius;
  var y1 = -radius;
  var y2 = radius;

  var horizontal_diff = abs(x2 - x1);
  var horizontal_steps = horizontal_diff/grid_steps;

  var vertial_diff = abs(y2 - y1);
  var vertiacal_steps = vertial_diff/grid_steps;

  push();
  textSize(20);
  fill(98,245,60);
  translate(width/2,height/2);
  textAlign(CENTER,CENTER);
  var text_step_for_draw = max_distance / grid_steps;
  var temp_text_step_for_draw = 0;

  var counter1 = 0;
  var i = 0;

  for(i = 0; i >= x1; i = i - vertiacal_steps){
    text(temp_text_step_for_draw,-i,radius*1.1);
    temp_text_step_for_draw = temp_text_step_for_draw + text_step_for_draw;
    counter1++;
  }

  temp_text_step_for_draw = 0;
  var counter2 = 0;
  var j = 0;
  for(j = 0; j <= x2; j = j + vertiacal_steps){
    text(temp_text_step_for_draw,-j,radius*1.1);
    temp_text_step_for_draw = temp_text_step_for_draw - text_step_for_draw;
    counter2++;
  }

  temp_text_step_for_draw = 0;
  var counter3 = 0;
  var k = 0;
  for(k = y2; k >= y1; k = k - vertiacal_steps){
    text(temp_text_step_for_draw,-radius*1.1,k);
    temp_text_step_for_draw = temp_text_step_for_draw + text_step_for_draw;
    counter3++;
  }
  if (counter3 < (grid_steps+1)){
    text(temp_text_step_for_draw,-radius*1.1,k);
  }
  
  pop();
}

function draw_sensor_stick(radius){
var px_distance = Number(distance_between_sensor_1_2)*2*radius/max_distance;

push();
  translate(width/2,height/2);
  strokeWeight(20);
  stroke("#FF5729");
  line(0,radius,((px_distance/2)),radius); // draws the line according to the angle
  line(0,radius,(-(px_distance/2)),radius); // draws the line according to the angle
pop();
}

function check_triangle_validity(a, b, c){

   if ((a + b <= c) || (a + c <= b) || (b + c <= a)){
        return false;
      }else{
        return true;
      }
}

function draw_control_box(w,h){
  var inputs_gap = 40;
  var start_input_position = 85; 
  label1 = createSpan('System Control');
  label1.position(25, 25);
  label1.style('color', '#FFFFFF');
  label1.style('font-size', '1.75em');
  input_box_size = '70px';
  //////////////////////////
  var control_div = createDiv('');
  control_div.position(20,75);
  control_div.style('border','1px solid #FFFFFF');
  control_div.style('width','15em');
  control_div.style('height', (h*.8)+'px');
  control_div.style('padding', '1em');
  //////////////////////////
  label2 = createSpan('Distance division');
  label2.position(35, start_input_position+(0*inputs_gap));  
  label2.style('color', '#FFFFFF');
  label2.style('margin-top', '1em');
  
  input2 = createInput();
  input2.position(35, start_input_position+(1*inputs_gap));
  input2.style('width', input_box_size);
  input2.class('form-control');
  input2.value(grid_steps);
  input2.input(update_button_2_color);
  
  button2 = createButton('GO');
  button2.position(input2.x+input2.width+5, input2.y);
  button2.class('btn btn-primary');
  button2.style('width', "3em");
  
  button2.mousePressed(update_grid_steps);
  //////////////////////////
  label3 = createSpan('Max distance (cm)');
  label3.position(35, start_input_position+(2*inputs_gap));  
  label3.style('color', '#FFFFFF');
  label3.style('margin-top', '1em');

  input3 = createInput();
  input3.position(35, start_input_position+(3*inputs_gap));
  input3.style('width', input_box_size);
  input3.class('form-control');
  input3.value(max_distance);
  input3.input(update_button_3_color);
  
  button3 = createButton('GO');
  button3.position(input3.x+input3.width+5, input3.y);
  button3.class('btn btn-primary');
  button3.style('width', "3em");
  
  button3.mousePressed(update_max_distance);
  
  //////////////////////////  
  label4 = createSpan('Sensor Gap (cm)');
  label4.position(35, start_input_position+(4*inputs_gap));  
  label4.style('color', '#FFFFFF');
  label4.style('margin-top', '1em');

  input4 = createInput();
  input4.position(35, start_input_position+(5*inputs_gap));
  input4.style('width', input_box_size);
  input4.class('form-control');
  input4.value(distance_between_sensor_1_2);
  input4.input(update_button_4_color);

  button4 = createButton('GO');
  button4.position(input4.x+input4.width+5, input4.y);
  button4.class('btn btn-primary');
  button4.style('width', "3em");
  
  button4.mousePressed(update_distance_between_sensor_1_2);
  //////////////////////////  
  label5 = createSpan('Left distance (cm)');
  label5.position(35, start_input_position+(6*inputs_gap));  
  label5.style('color', '#FFFFFF');
  label5.style('margin-top', '1em');

  input5 = createInput();
  input5.position(35, start_input_position+(7*inputs_gap));
  input5.style('width', input_box_size);
  input5.class('form-control');
  input5.value(left_distance);
  input5.input(update_button_5_color);

  button5 = createButton('GO');
  button5.position(input5.x+input5.width+5, input5.y);
  button5.class('btn btn-primary');
  button5.style('width', "3em");
  
  button5.mousePressed(update_left_distance);
  //////////////////////////  
  label6 = createSpan('Right distance (cm)');
  label6.position(35, start_input_position+(8*inputs_gap));  
  label6.style('color', '#FFFFFF');
  label6.style('margin-top', '1em');

  input6 = createInput();
  input6.position(35, start_input_position+(9*inputs_gap));
  input6.style('width', input_box_size);
  input6.class('form-control');
  input6.value(right_distance);
  input6.input(update_button_6_color);

  button6 = createButton('GO');
  button6.position(input6.x+input6.width+5, input6.y);
  button6.class('btn btn-primary');
  button6.style('width', "3em");
  
  button6.mousePressed(update_right_distance);
  //////////////////////////
  button7 = createButton('Show on 2D Plane');
  button7.position(35, start_input_position+(15*inputs_gap));
  button7.class('btn btn-success');
  
  button7.mousePressed(show_on_2d_plane);
  //////////////////////////
  label8 = createSpan('Sensor Coverage ('+sensor_angle_coverage+'°)');
  label8.position(35, start_input_position+(10*inputs_gap));  
  label8.style('color', '#FFFFFF');
  label8.style('margin-top', '1em');
  slider_1 = createSlider(0, 180, sensor_angle_coverage);
  slider_1.position(35, start_input_position+(11*inputs_gap));
  //////////////////////////
  label9 = createSpan('Sensor angle ('+sensor_angle+'°)');
  label9.position(35, start_input_position+(12*inputs_gap));  
  label9.style('color', '#FFFFFF');
  label9.style('margin-top', '1em');
  slider_2 = createSlider(0, 180, sensor_angle);
  slider_2.position(35, start_input_position+(13*inputs_gap));
  //////////////////////////
  input10 = createCheckbox(': Show sensor coverage', show_sensor_coverage);
  input10.value( show_sensor_coverage);
  input10.position(35, start_input_position+(14*inputs_gap));
  input10.style('color', "#FFFFFF");
  input10.changed(update_show_sensor_coverage);
  //////////////////////////
  button11 = createButton('');
  button11.position(35, start_input_position+(16*inputs_gap)+10);
  button11.class('btn btn-success');
  button11.style('width',(button7.width+10)+'px');
  button11.html("Start");
  button11.mousePressed(update_start_reading);
  //////////////////////////
  input10 = createCheckbox(': Show object lines', show_object_lines);
  input10.value( show_object_lines);
  input10.position(35, start_input_position+(17*inputs_gap));
  input10.style('color', "#FFFFFF");
  input10.style('margin-top', "1em");
  input10.changed(update_show_object_lines);

  /*&&&&&&&&&&&&&&&&&&&&&&&&&*/
  label_r_1 = createSpan('System Report');
  label_r_1.position(width*0.82, 25);
  label_r_1.style('color', '#FFFFFF');
  label_r_1.style('font-size', '1.75em');
  input_box_size = '70px';
  //////////////////////////
  var control_r_div = createDiv('');
  control_r_div.position(width*0.8,75);
  control_r_div.style('border','1px solid #FFFFFF');
  control_r_div.style('width','15em');
  control_r_div.style('height', (h*.8)+'px');
  control_r_div.style('padding', '1em');
  //////////////////////////
  label_r_2 = createSpan('Invalid triangle');
  label_r_2.position(width*0.82, start_input_position);  
  label_r_2.style('color', '#FFFFFF');
  label_r_2.style('font-size', '1.25em');
  label_r_2.style('background-color', '#EB0700');
  label_r_2.style('padding', '0.25em 1.5em 0.25em 1.5em');
  //////////////////////////
  label_r_3 = createSpan('Cartesian (cm)');
  label_r_3.position(width*0.82, start_input_position+(2*inputs_gap));  
  label_r_3.style('color', '#FFFFFF');
  label_r_3.style('font-size', '1.75em');
  //////////////////////////
  label_r_4 = createSpan('x : 0.0');
  label_r_4.position(width*0.82, start_input_position+(3*inputs_gap));  
  label_r_4.style('color', '#FFFFFF');
  label_r_4.style('font-size', '1.75em');
  //////////////////////////
  label_r_5 = createSpan('y : 0.0');
  label_r_5.position(width*0.82, start_input_position+(4*inputs_gap));  
  label_r_5.style('color', '#FFFFFF');
  label_r_5.style('font-size', '1.75em');
  //////////////////////////
  label_r_6 = createSpan('Polar (cm, deg)');
  label_r_6.position(width*0.82, start_input_position+(6*inputs_gap));  
  label_r_6.style('color', '#FFFFFF');
  label_r_6.style('font-size', '1.75em');
  //////////////////////////
  label_r_7 = createSpan('r : 0.0');
  label_r_7.position(width*0.82, start_input_position+(7*inputs_gap));  
  label_r_7.style('color', '#FFFFFF');
  label_r_7.style('font-size', '1.75em');
  //////////////////////////
  label_r_8 = createSpan('θ : 0.0');
  label_r_8.position(width*0.82, start_input_position+(8*inputs_gap));  
  label_r_8.style('color', '#FFFFFF');
  label_r_8.style('font-size', '1.75em');
  //////////////////////////
  label_r_9 = createSpan('Sensor info');
  label_r_9.position(width*0.82, start_input_position+(10*inputs_gap));  
  label_r_9.style('color', '#FFFFFF');
  label_r_9.style('font-size', '1.75em');
  //////////////////////////
  label_r_10 = createSpan('Max: '+sensor_max_distance);
  label_r_10.position(width*0.82, start_input_position+(11*inputs_gap));  
  label_r_10.style('color', '#FFFFFF');
  label_r_10.style('font-size', '1.75em');
  //////////////////////////
  label_r_11 = createSpan('min: '+sensor_min_distance);
  label_r_11.position(width*0.82, start_input_position+(12*inputs_gap));  
  label_r_11.style('color', '#FFFFFF');
  label_r_11.style('font-size', '1.75em');

  }



  function update_grid_steps(){
  	grid_steps = Number(input2.value());
  	if(isNaN(grid_steps)){
  		grid_steps = 10;
  	}
  	if(grid_steps === 0){
  		grid_steps = 10;	
  	}
    button2.class("btn btn-primary");
  }

  function update_max_distance(){
    max_distance = Number(input3.value());
    if(isNaN(max_distance)){
      max_distance = 100;
    }
    if(max_distance === 0){
      max_distance = 100;  
    }
    button3.class("btn btn-primary");
  }

function get_radius() {
  var radius = 0;
  if (width <= height) {
    radius = (width/2)*0.85;
  } else {
    radius = (height/2)*0.85;
  }
  return radius;
}

function update_distance_between_sensor_1_2(){
  distance_between_sensor_1_2 = Number(input4.value());
  button4.class("btn btn-primary");
}

function update_left_distance(){
  left_distance = Number(input5.value());
  button5.class("btn btn-primary");
}
function update_right_distance(){
 right_distance = Number(input6.value());
 button6.class("btn btn-primary");
}

function update_button_2_color() {
  button2.class("btn btn-danger");
}

function update_button_3_color() {
  button3.class("btn btn-danger");
}

function update_button_4_color() {
  button4.class("btn btn-danger");
}

function update_button_5_color() {
  button5.class("btn btn-danger");
}

function update_button_6_color() {
  button6.class("btn btn-danger");
}

function show_on_2d_plane(){
  input5.value(left_distance);
  input6.value(right_distance);
  var validiry_result = check_triangle_validity(left_distance, right_distance, distance_between_sensor_1_2);
  
  if (validiry_result){
    label_r_2.html("Valid triangle&nbsp;&nbsp;&nbsp;");
    label_r_2.style('background-color', '#049F40');

  var left_sensor_angle = Math.acos((Math.pow(left_distance,2)+ Math.pow(distance_between_sensor_1_2,2)- Math.pow(right_distance,2))/(2*left_distance*distance_between_sensor_1_2));
  var right_sensor_angle = Math.acos((Math.pow(right_distance,2)+ Math.pow(distance_between_sensor_1_2,2)- Math.pow(left_distance,2))/(2*right_distance*distance_between_sensor_1_2));

  var px_distance_between_sensor_1_2 = Number(distance_between_sensor_1_2)*2*radius/max_distance;
  var px_left_distance = Number(left_distance)*2*radius/max_distance;
  var px_right_distance = Number(right_distance)*2*radius/max_distance;
  
  left_sensor_x1 = 0;
  left_sensor_x2 = px_left_distance*Math.cos(left_sensor_angle);
  left_sensor_y1 = 0;
  left_sensor_y2 = -px_left_distance*Math.sin(left_sensor_angle);
  r = (1/2)*Math.sqrt((2*Math.pow(left_distance,2)+2*Math.pow(right_distance,2)-Math.pow(distance_between_sensor_1_2,2)));
  
  theta = Math.acos((Math.pow((r),2)+Math.pow((distance_between_sensor_1_2/2),2)-Math.pow(right_distance,2))/(2*(distance_between_sensor_1_2/2)*r));
  
  x = r*cos(theta);
  y = r*sin(theta);
  

  right_sensor_x1 = 0;
  right_sensor_x2 = -px_right_distance*Math.cos(right_sensor_angle);
  right_sensor_y1 = 0;
  right_sensor_y2 = -px_right_distance*Math.sin(right_sensor_angle);
  
  object_x = px_left_distance*Math.cos(left_sensor_angle);
  object_y = -px_left_distance*Math.sin(left_sensor_angle);
  }else{

    label_r_2.html("Invalid triangle");
    label_r_2.style('background-color', '#EB0700');
    x = 0;
    y = 0;
    r = 0;
    theta = 0;
    right_sensor_x1 = 0;
    right_sensor_x2 = 0;
    right_sensor_y1 = 0;
    right_sensor_y2 = 0;
    left_sensor_x1 = 0;
    left_sensor_x2 = 0;
    left_sensor_y1 = 0;
    left_sensor_y2 = 0;
    object_x = 0;
    object_y = 0;
  }

  label_r_7.html('r : '+Math.round(r));
  label_r_8.html('θ : '+Math.round(theta*180/(Math.PI))+'°');
  label_r_4.html('x : '+Math.round(x));
  label_r_5.html('y : '+Math.round(y));
}

function draw_sensor_coverage(radius){
  var px_distance = Number(distance_between_sensor_1_2)*2*radius/max_distance;
  var inner_side_length = (px_distance/2)/abs(cos(radians(sensor_angle-(sensor_angle_coverage/2))));
  var middle_side_length = (px_distance/2)/abs(cos(radians(sensor_angle)));
  var outer_side_length = (px_distance/2)/abs(cos(radians(sensor_angle+(sensor_angle_coverage/2))));
  
  var cm_inner_side_length = (Number(distance_between_sensor_1_2)/2)/abs(cos(radians(sensor_angle-(sensor_angle_coverage/2))));
  var cm_outer_side_length = (Number(distance_between_sensor_1_2)/2)/abs(cos(radians(sensor_angle+(sensor_angle_coverage/2))));
  var min_coverage = Math.sqrt(Math.pow(cm_inner_side_length,2)-Math.pow(distance_between_sensor_1_2/2,2));
  var max_coverage = Math.sqrt(Math.pow(cm_outer_side_length,2)-Math.pow(distance_between_sensor_1_2/2,2));
  
  if(show_sensor_coverage){
  push();
  translate(width/2,(height/2)+radius);
  push();
  strokeWeight(5);
  
  translate((-(px_distance/2)),0);
  rotate((-sensor_angle)* PI / 180);
  push();
  noStroke();
  rect(0,-50,50,100, 20);
  pop();
  stroke(30,250,60);
  line(0,0,(middle_side_length)*(cos(radians(current_angle-90))),-(middle_side_length)*(sin(radians(current_angle-90)))); 
  stroke(226,255,0);
  line(0,0,(inner_side_length)*(cos(radians(current_angle-(sensor_angle_coverage/2)-90))),-(inner_side_length)*(sin(radians(current_angle-(sensor_angle_coverage/2)-90)))); 
  stroke(255,0,0);
  line(0,0,(outer_side_length)*(cos(radians(current_angle+(sensor_angle_coverage/2)-90))),-(outer_side_length)*(sin(radians(current_angle+(sensor_angle_coverage/2)-90))));

  pop();
  
  push();
  strokeWeight(5);
  translate(((px_distance/2)),0);
  rotate(sensor_angle* PI / 180);
  push();
  noStroke();
  rect(-50,-50,50,100, 20);
  pop();
  stroke(30,250,60);
  line(0,0,(middle_side_length)*(cos(radians(current_angle+90))),-(middle_side_length)*(sin(radians(current_angle+90)))); 
  stroke(226,255,0);
  line(0,0,(inner_side_length)*(cos(radians(current_angle+(sensor_angle_coverage/2)+90))),-(inner_side_length)*(sin(radians(current_angle+(sensor_angle_coverage/2)+90))));  
  stroke(255,0,0);
  line(0,0,(outer_side_length)*(cos(radians(current_angle-(sensor_angle_coverage/2)+90))),-(outer_side_length)*(sin(radians(current_angle-(sensor_angle_coverage/2)+90))));

  pop();
  pop();
  }
  sensor_min_distance = min_coverage;
  sensor_max_distance = max_coverage;

}

function update_show_sensor_coverage(){
  show_sensor_coverage = !show_sensor_coverage;
}

function sendIntro() {
  // convert the message object to a string and send it:
  //socket.send("Hello");
}

function readMessage(event) {

  var msg = JSON.parse(event.data);

  if(msg["id"] == 3){ // For the servo location
    if(start_reading){
    serialEventHandler(msg.value);
    }
  } /*else if (msg["id"] == 4){ //For the distance
    serialEventHandler2(msg.value);
  }*/
}

function serialEventHandler(myPort) { // starts reading data from the Serial Port

  var data = myPort;

  var last_char = data.substr(data.length - 1);

  if(last_char === '~'){
    data = data.slice(0,-1);
  }
  if(isJson(data)){
    json_data = JSON.parse(data);

    if(json_data["center_servo_angle"]){
      center_servo_current_angle = Number(parseInt(json_data["center_servo_angle"]));
    }
    if(json_data["left_distance"]){
      left_distance = Number(parseFloat(json_data["left_distance"])) * 2.54;
    }
    if(json_data["right_distance"]){
      right_distance = Number(parseFloat(json_data["right_distance"])) * 2.54;
    }
    if(json_data["run_center_servo"]){
      center_servo_status = Number(parseInt(json_data["run_center_servo"]));
    }

    if(json_data["sensor_servo_angle"]){
       sensor_angle = Number(parseInt(json_data["sensor_servo_angle"])); 
      slider_2.value(Number(sensor_angle));
      label9.html('Sensor angle ('+sensor_angle+'°)');
    }
    
  }else {
    console.log("No JSON");
    console.log(data);
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

function update_start_reading(){
  start_reading = !start_reading;
    if(!start_reading){
  button11.class('btn btn-success');
  button11.html("Start");
  window_fill = 50;
  }else{
    button11.class('btn btn-danger');
    button11.html("Stop");
    window_fill = 4;
  }
  }
  function update_show_object_lines(){
    show_object_lines = !show_object_lines;
  }