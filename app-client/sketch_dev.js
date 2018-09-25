/*
 * California State University Northridge
 * ECE 493 Senior Design
 * Project: Ultrasonic radar
 * Participants: Saba Janamian, Hambartzum Gamburian, Giovanni Alonzo, Hamed Seyedroudbari, Andrew Zaragoza, Xiaoao Feng
 * Advisor: Dr. Shahnam Mirzaei
 * Code Description: P5js applicaiton to draw a radar graph in the browser
 * Author: Saba Janamian
 * Version: 1.0
 * Date: 3/7/2018
 */

var number_of_circles = 5;
var show_distance_division = true;
//--------------------
var distance_text_step = 5;
var show_text_for_distance_division = true;
//--------------------
var number_of_lines = 8;
var show_angle_division = true;
//--------------------
var angle_major_division_step = 360/5;
var show_angle_major_division_step = true;
//--------------------
var angle_minor_division_step = 360;
var show_angle_minor_division_step = true;
//--------------------
var angle_text_step = 8;
var show_angle_text_step = true;
//--------------------
var max_distance = 200;
//--------------------

// Global varaibales 
var current_angle = 0.0;
var object_angle = 0.0;
var current_distance = 0.0;
var initial_reading = 0; 
var distance_between_sensor_1_2 = 20;
var sensor_angle = 90;

var show_sensor_axis = false;
var show_sensor_caverage = false;



var distance_unit = "cm";
 
var input4;
var object_array = [];

var button_label_for_center_servo = "Start";
var center_servo_status = 0;

var gyro_div_center_x;
var gyro_div_center_y;

var sensor_upper_level = 0;
var sensor_mid_level = 0;
var sensor_lower_level = 0;

var host = 'localhost:8888';
var socket; // the websocket
var sensorValue = 0; // the

var fade_angle = 60;

function set_current_angle(){
current_angle = current_angle + 1;
if(current_angle >= 360){
   current_angle = 0; 
  }
}


function setup(){ 
  createCanvas(window.innerWidth, window.innerHeight);
  //pixelDensity(1);
  smooth();
  draw_control_box(window.innerWidth, window.innerHeight);

  socket = new WebSocket('ws://' + host+'/ws');
  // socket connection listener:
  socket.onopen = sendIntro;
  // socket message listener:
  socket.onmessage = readMessage;
 }

function draw(){ 
  var radius = get_radius();
  draw_window();
  set_up_radar_graphic();
  
  if(show_distance_division){
  	draw_circle(number_of_circles, radius);
	}
  
  if (show_text_for_distance_division){
  draw_distance_text(distance_text_step, radius, max_distance, distance_unit);
	}
  
  if(show_angle_division){
  	draw_line(number_of_lines, radius);
	}

	if (show_angle_minor_division_step){
  draw_sub_minor_div_line(angle_minor_division_step,radius);
  }

  if (show_angle_major_division_step){
  	draw_sub_major_div_line(angle_major_division_step,radius);
	}

	if (show_angle_text_step){
  	draw_angle_text(angle_text_step, radius);
  	}

  draw_radar_arrow(radius);
  if(show_sensor_axis){
  draw_radar_stick(radius);
  }
  if(show_sensor_caverage){
   draw_sensor_coverage(radius); 
  }

  if(center_servo_status == 1){
  set_current_angle();
  }
  //current_distance = getRndInteger(10, max_distance);
  draw_object(object_angle, current_distance, radius, max_distance);
 for(var i=0; i < object_array.length; i++){
  if (abs(current_angle-object_array[i].angle) > fade_angle){
    object_array.splice(i,1);
  }else {
  object_array[i].draw();            
  }
}

}

function sendIntro() {
  // convert the message object to a string and send it:
  //socket.send("Hello");
}

function readMessage(event) {

  var msg = JSON.parse(event.data);
  // console.log(event);
  if(msg["id"] == 3){ // For the servo location
    serialEventHandler(msg.value);
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
    //console.log(json_data);
    var left_distance = 0;
    var right_distance = 0;

    if(json_data["center_servo_angle"]){
      current_angle = Number(parseInt(json_data["center_servo_angle"]));
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
    }

    
    console.log("distance_between_sensor_1_2",distance_between_sensor_1_2);
    valid_triangle = check_triangle_validity(left_distance, right_distance, Number(distance_between_sensor_1_2));
    if(valid_triangle){
      var dist_obj = calculate_distance(left_distance, right_distance, Number(distance_between_sensor_1_2));
      //object_angle = dist_obj["angle"]+current_angle;
      object_angle = current_angle;
      current_distance = dist_obj["distance"];
      if(current_distance > max_distance) {
      object_angle = 0;
      current_distance = 0;  
      }
      console.log("valid_triangle",valid_triangle,dist_obj,"angle",object_angle,"current_distance",current_distance );
    } else {
      console.log("Not valid Triangle:",left_distance,right_distance, Number(distance_between_sensor_1_2));
      object_angle = 0;
      current_distance = 0;
    }

  }else {
    console.log("No JSON");
    console.log(data);
  }

}



function calculate_distance(left_distance, right_distance, distance_between_sensor){
  var s = (left_distance + right_distance + distance_between_sensor )/2;
  var area = Math.sqrt(s*(s-left_distance)*(s-right_distance)*(s-distance_between_sensor));
  var h = 2*area/distance_between_sensor;
  var m = (1/2)*Math.sqrt((2*Math.pow(left_distance,2))+(2*Math.pow(right_distance,2))-(Math.pow(distance_between_sensor,2)));
  var angle = Math.asin(h/m) * 180/Math.PI;
  var result = {"distance":m, "angle":angle};
  console.log("-->",s,area,h,m,angle,left_distance, right_distance, distance_between_sensor,result); 
  return result;
}

function check_triangle_validity(a, b, c){
   if ((a + b <= c) || (a + c <= b) || (b + c <= a)){
        return false;
      }else{
        return true;
      }
}

function set_up_radar_graphic(){
  translate(width/2,height/2); // Move the coordinate to the center of the window.
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

function draw_window(){
  push();
  // simulating motion blur and slow fade of the moving line
  fill(98,245,31);
  noStroke();
  fill(0,4); 
  rect(0, 0, width, height);
  pop();
}

function draw_circle(number_of_circles, radius) {
  push();
  noFill();
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

function draw_line(number_of_lines, radius){
  push();
  stroke(98,245,31); // green color
  var line_step = 360 / number_of_lines;
  var temp_line_step = 0.00;
  for (var i=0; i < number_of_lines; i++){
    line(0,0,(radius)*cos(radians(temp_line_step)),-(radius)*sin(radians(temp_line_step)));
    temp_line_step = temp_line_step+line_step;
  }
  pop();
}

function draw_sub_minor_div_line(number_of_lines, radius){
  push();
  stroke(98,245,31); // green color
  var line_step = 360 / number_of_lines;
  var temp_line_step = 0.00;
  for (var i=0; i < number_of_lines; i++){
    line((radius)*cos(radians(temp_line_step)),-(radius)*sin(radians(temp_line_step)),(radius-20)*cos(radians(temp_line_step)),-(radius-20)*sin(radians(temp_line_step)));
    temp_line_step = temp_line_step+line_step;
  }
  pop();
}

function draw_sub_major_div_line(number_of_lines, radius){
  push();
  stroke(98,245,31); // green color
  if (number_of_lines == 0){
  	number_of_lines = 1;
  }
  var line_step = 360 / number_of_lines;
  var temp_line_step = 0.00;
  for (var i=0; i < number_of_lines; i++){
    line((radius)*cos(radians(temp_line_step)),-(radius)*sin(radians(temp_line_step)),(radius-40)*cos(radians(temp_line_step)),-(radius-40)*sin(radians(temp_line_step)));
    temp_line_step = temp_line_step+line_step;
  }
  pop();
}

function draw_angle_text(number_of_text, radius) { // draws the texts on the screen
  textSize(25);
  fill(98,245,60);
  
  var text_step = 360 / number_of_text;
  var temp_text_step = 0.00;

  temp_text_step = 0.00;
  for (var i=0; i < number_of_text; i++){
    //if(cos(radians(temp_text_step)) > 0){
     push();
     translate((radius)*cos(radians(temp_text_step)),-(radius)*sin(radians(temp_text_step)));
     //translate((radius)*cos(radians(temp_text_step))*1.05,-(radius)*sin(radians(temp_text_step))*1.1);
     rotate(radians(90-temp_text_step));
     textAlign(CENTER);
     text(temp_text_step+"°",0,-10);
     pop();
    //}else {
    // text(temp_text_step+"°",(radius)*cos(radians(temp_text_step))*1.3,-(radius)*sin(radians(temp_text_step))*1.1);
    //}
    temp_text_step = round_int(temp_text_step+text_step,1);
}
}


function draw_distance_text(number_of_text, radius, max_distance, distance_unit) { // draws the texts on the screen
  
  textSize(15);
  fill(98,245,60);
  var text_step = 100 / number_of_text;
  var temp_text_step = 0.00;
  var text_step_for_draw = max_distance / number_of_text;
  var temp_text_step_for_draw = 0.0;
  temp_text_step = 0.00;
  for (var i=0; i < number_of_text; i++){
    temp_text_step = temp_text_step+text_step;
    temp_text_step_for_draw = round_int(temp_text_step_for_draw + text_step_for_draw, 1);
    
    push();
    translate(radius*(temp_text_step/100)*1.02,height*0.02);
    rotate(-radians(-45));
    text(temp_text_step_for_draw+" "+distance_unit,0,0);
    pop();

    push();
    translate(width*0.01,-radius*(temp_text_step/100)*1.02);
    //rotate(-radians(-45));
    text(temp_text_step_for_draw+" "+distance_unit,0,0);
    pop();

  }
  
}

function draw_radar_arrow(radius){
  push();
  strokeWeight(12);
  stroke(30,250,60);
  line(0,0,(radius*0.98)*cos(radians(current_angle)),-(radius*0.98)*sin(radians(current_angle))); // draws the line according to the angle
  pop();
}


function draw_radar_stick(radius){
  var px_distance = Number(distance_between_sensor_1_2)*radius/max_distance;
  var inner_side_length = (px_distance/2)/abs(cos(radians(sensor_angle-15)));
  var middle_side_length = (px_distance/2)/abs(cos(radians(sensor_angle)));
  var outer_side_length = (px_distance/2)/abs(cos(radians(sensor_angle+15)));
  //console.log(inner_side_length, middle_side_length,outer_side_length,sensor_angle,px_distance/2 );
  push();
  strokeWeight(10);
  stroke(39,137,203);
  line(0,0,((px_distance/2)*0.98)*cos(radians(current_angle+90)),-((px_distance/2)*0.98)*sin(radians(current_angle+90))); // draws the line according to the angle
  line(0,0,((px_distance/2)*0.98)*cos(radians(current_angle-90)),-((px_distance/2)*0.98)*sin(radians(current_angle-90))); // draws the line according to the angle
  pop();
}

function draw_sensor_coverage(radius){
  var px_distance = Number(distance_between_sensor_1_2)*radius/max_distance;
  // Calculate the distance in pixcel
  var inner_side_length = (px_distance/2)/abs(cos(radians(sensor_angle-15)));
  var middle_side_length = (px_distance/2)/abs(cos(radians(sensor_angle)));
  var outer_side_length = (px_distance/2)/abs(cos(radians(sensor_angle+15)));

  // Calculate the distance in cm
  var inner_side_length_in_cm = (Number(distance_between_sensor_1_2)/2)/abs(cos(radians(sensor_angle-15)));
  height_inner = Math.sqrt(Math.pow(inner_side_length_in_cm,2) - Math.pow((Number(distance_between_sensor_1_2)/2),2));

  // Calculate the distance in cm
  var middle_side_length_in_cm = (Number(distance_between_sensor_1_2)/2)/abs(cos(radians(sensor_angle)));
  height_middle = Math.sqrt(Math.pow(middle_side_length_in_cm,2) - Math.pow((Number(distance_between_sensor_1_2)/2),2));
  
  // Calculate the distance in cm
  var outer_side_length_in_cm = (Number(distance_between_sensor_1_2)/2)/abs(cos(radians(sensor_angle+15)));
  height_outer = Math.sqrt(Math.pow(outer_side_length_in_cm,2) - Math.pow((Number(distance_between_sensor_1_2)/2),2));
 
  console.log(height_inner,height_middle,height_outer);

  push();
  strokeWeight(2);
  
  translate(((px_distance/2)*0.98)*cos(radians(current_angle+90)),-((px_distance/2)*0.98)*sin(radians(current_angle+90)));
  rotate((-sensor_angle)* PI / 180);
  stroke(30,250,60);
  line(0,0,(middle_side_length*0.98)*(cos(radians(current_angle-90))),-(middle_side_length*0.98)*(sin(radians(current_angle-90)))); 
  stroke(226,255,0);
  line(0,0,(inner_side_length*0.98)*(cos(radians(current_angle-15-90))),-(inner_side_length*0.98)*(sin(radians(current_angle-15-90)))); 
  stroke(255,0,0);
  line(0,0,(outer_side_length*0.98)*(cos(radians(current_angle+15-90))),-(outer_side_length*0.98)*(sin(radians(current_angle+15-90))));
  pop();
  
  push();
  strokeWeight(2);
  translate(((px_distance/2)*0.98)*cos(radians(current_angle-90)),-((px_distance/2)*0.98)*sin(radians(current_angle-90)));
  rotate(sensor_angle* PI / 180);
  stroke(30,250,60);
  line(0,0,(middle_side_length*0.98)*(cos(radians(current_angle+90))),-(middle_side_length*0.98)*(sin(radians(current_angle+90)))); 
  stroke(226,255,0);
  line(0,0,(inner_side_length*0.98)*(cos(radians(current_angle+15+90))),-(inner_side_length*0.98)*(sin(radians(current_angle+15+90))));  
  stroke(255,0,0);
  line(0,0,(outer_side_length*0.98)*(cos(radians(current_angle-15+90))),-(outer_side_length*0.98)*(sin(radians(current_angle-15+90))));
  //line(((px_distance/2)*0.98)*cos(radians(current_angle+90)),-((px_distance/2)*0.98)*sin(radians(current_angle+90)),(radius*0.98)*(cos(radians(current_angle))),-(radius*0.98)*(sin(radians(current_angle)))); 
  //line(((px_distance/2)*0.98)*cos(radians(current_angle-90)),-((px_distance/2)*0.98)*sin(radians(current_angle-90)),(radius*0.98)*cos(radians(current_angle)),-(radius*0.98)*sin(radians(current_angle))); 
  pop();  
}

function draw_object(angle, distance, radius, max_distance) {
  push();

  var px_distance = distance*radius/ max_distance; // distance from the sensor to the object in pixels
  //console.log(distance, radius, max_distance, px_distance);
  object_array.push(new CircleClass( px_distance*cos(radians(angle)), px_distance*sin(-radians(angle )), frameCount, angle, distance ) );
  pop();
}



// DOM elements functions
function draw_control_box(w,h){
  var inputs_gap = 40;
  var start_input_position = 85; 
  label1 = createSpan('System Control');
  label1.position(25, 25);
  label1.style('color', '#FFFFFF');
  label1.style('font-size', '1.75em');
  input_box_size = '40px';
  //////////////////////////
  var control_div = createDiv('');
  control_div.position(20,75);
  control_div.style('border','1px solid #FFFFFF');
  control_div.style('width','15em');
  control_div.style('height', (h*.8)+'px');
  control_div.style('padding', '1em');
  //////////////////////////
//////////////////////////
  input2 = createInput();
  input2.position(35, start_input_position+(0*inputs_gap));
  input2.style('width', input_box_size);
  input2.input(update_circle_number);
  input2.value(number_of_circles);

  label2 = createSpan('Distance division (cm)');
  label2.position(input2.x+input2.width+10, input2.y+2);
  label2.style('color', '#FFFFFF');
  //////////////////////////
  input3 = createInput();
  input3.position(35, start_input_position+(1*inputs_gap));
  input3.style('width', input_box_size);
  input3.input(update_line_number);
  input3.value(number_of_lines);

  label3 = createSpan('Angle division (Deg)');
  label3.position(input3.x+input3.width+10, input3.y+2);
  label3.style('color', '#FFFFFF');
  //////////////////////////
  input4 = createInput();
  input4.position(35, start_input_position+(2*inputs_gap));
  input4.style('width', input_box_size);
  input4.input(update_max_distance);
  input4.value(max_distance);

  label4 = createSpan('Range (cm)');
  label4.position(input4.x+input4.width+10, input4.y+2);
  label4.style('color', '#FFFFFF');
  //////////////////////////
  input5 = createInput();
  input5.position(35, start_input_position+(3*inputs_gap));
  input5.style('width', input_box_size);
  input5.input(update_distance_between_sensor_1_2);
  input5.value(distance_between_sensor_1_2);

  label5 = createSpan('Sensors gap (cm)');
  label5.position(input5.x+input5.width+10, input5.y+2);
  label5.style('color', '#FFFFFF');
  //////////////////////////
  input6 = createInput();
  input6.position(35, start_input_position+(4*inputs_gap));
  input6.style('width', input_box_size);
  input6.input(update_sensor_angle);
  input6.value(sensor_angle);

  label6 = createSpan('Sensors angle [15-90]');
  label6.position(input6.x+input6.width+10, input6.y+2);
  label6.style('color', '#FFFFFF');
  //////////////////////////
  input7 = createButton('');
  input7.position(35, start_input_position+(11*inputs_gap));
  if(center_servo_status == 1){
  input7.class('btn btn-danger');
  input7.html("Stop");
  }else {
    input7.class('btn btn-success');
    input7.html("Start");
  }
  input7.style('width', "13em");
  input7.mousePressed(update_servo_status);
  input7.value(center_servo_status);
  

  //////////////////////////
  input8 = createButton("Show Gyro");
  input8.position(35, start_input_position+(7*inputs_gap));
  input8.class('btn btn-primary');
  input8.style('width', "13em");
  input8.mousePressed(show_gyro_window);
  input8.value(0);
  input8.html("Show Gyro");

    //////////////////////////
  input9 = createButton("Show Compass");
  input9.position(35, start_input_position+(8*inputs_gap));
  input9.class('btn btn-primary');
  input9.style('width', "13em");
  input9.mousePressed(show_compass_window);
  input9.value(0);
  input9.html("Show Compass");

  //////////////////////////
  input10 = createCheckbox(':  Show sensor axis', show_sensor_axis);
  input10.value(show_sensor_axis);
  input10.position(35, start_input_position+(5*inputs_gap));
  input10.style('color', "#FFFFFF");
  input10.changed(update_show_sensor_axis);

  //////////////////////////
  input11 = createCheckbox(': Show sensor coverage', show_sensor_axis);
  input11.value(show_sensor_axis);
  input11.position(35, start_input_position+(6*inputs_gap));
  input11.style('color', "#FFFFFF");
  input11.changed(update_show_sensor_coverage);

  /*
  input2 = createInput();
  input2.position(35, start_input_position+(0*inputs_gap));
  input2.style('width', input_box_size);
  input2.input(update_circle_number);
  input2.value(number_of_circles);

  label2 = createSpan('Distance division step');
  label2.position(input2.x+input2.width+10, input2.y+2);
  label2.style('color', '#FFFFFF');
  //////////////////////////
  input3 = createCheckbox(':  Show distance division', show_distance_division);
  input3.value(show_distance_division);
  input3.position(35, start_input_position+(1*inputs_gap));
  input3.style('color', "#FFFFFF");
  input3.changed(update_show_distance_division);
  //////////////////////////
  input4 = createInput();
  input4.position(35, start_input_position+(2*inputs_gap));
  input4.style('width', input_box_size);
  input4.input(update_distance_text_step);
  input4.value(distance_text_step);

  label4 = createSpan('Distance text step');
  label4.position(input4.x+input4.width+10, input4.y+2);
  label4.style('color', '#FFFFFF');
  //////////////////////////
  input5 = createCheckbox(':  Show distance division text', show_text_for_distance_division);
  input5.value(show_text_for_distance_division);
  input5.position(35, start_input_position+(3*inputs_gap));
  input5.style('color', "#FFFFFF");
  input5.changed(update_show_text_for_distance_division);
  //////////////////////////
  input6 = createInput();
  input6.position(35, start_input_position+(4*inputs_gap));
  input6.style('width', input_box_size);
  input6.input(update_line_number);
  input6.value(number_of_lines);

  label6 = createSpan('Angle division step');
  label6.position(input6.x+input6.width+10, input6.y+2);
  label6.style('color', '#FFFFFF');
  //////////////////////////
  input7 = createCheckbox(':  Show Angle distance', show_angle_division);
  input7.value(show_angle_division);
  input7.position(35, start_input_position+(5*inputs_gap));
  input7.style('color', "#FFFFFF");
  input7.changed(update_show_angle_division);
  //////////////////////////
  input8 = createInput();
  input8.position(35, start_input_position+(6*inputs_gap));
  input8.style('width', input_box_size);
  input8.input(update_angle_major_division_step);
  input8.value(angle_major_division_step);

  label8 = createSpan('Angle major division step');
  label8.position(input8.x+input8.width+10, input8.y+2);
  label8.style('color', '#FFFFFF');
  //////////////////////////
  input7 = createCheckbox(':  Show angle major division', show_angle_major_division_step);
  input7.value(show_angle_major_division_step);
  input7.position(35, start_input_position+(7*inputs_gap));
  input7.style('color', "#FFFFFF");
  input7.changed(update_show_angle_major_division_step);
//////////////////////////
  input9 = createInput();
  input9.position(35, start_input_position+(8*inputs_gap));
  input9.style('width', input_box_size);
  input9.input(update_angle_minor_division_step);
  input9.value(angle_minor_division_step);

  label9 = createSpan('Angle minor division step');
  label9.position(input9.x+input9.width+10, input9.y+2);
  label9.style('color', '#FFFFFF');
  ////////////////////////
  input10 = createCheckbox(':  Show angle minor division', show_angle_minor_division_step);
  input10.value(show_angle_minor_division_step);
  input10.position(35, start_input_position+(9*inputs_gap));
  input10.style('color', "#FFFFFF");
  input10.changed(update_show_angle_minor_division_step);
  
  //////////////////////////
  input11 = createInput();
  input11.position(35, start_input_position+(10*inputs_gap));
  input11.style('width', input_box_size);
  input11.input(update_angle_text_step);
  input11.value(angle_text_step);

  label11 = createSpan('Angle text step');
  label11.position(input11.x+input11.width+10, input11.y+2);
  label11.style('color', '#FFFFFF');
  ////////////////////////
  input12 = createCheckbox(':  Show angle text', show_angle_text_step);
  input12.value(show_angle_text_step);
  input12.position(35, start_input_position+(11*inputs_gap));
  input12.style('color', "#FFFFFF");
  input12.changed(update_show_angle_text_step);
  ////////////////////////
  input13 = createInput();
  input13.position(35, start_input_position+(12*inputs_gap));
  input13.style('width', input_box_size);
  input13.input(update_max_distance);
  input13.value(max_distance);

  label13 = createSpan('Range (cm)');
  label13.position(input13.x+input13.width+10, input13.y+2);
  label13.style('color', '#FFFFFF');

  //////////////////////////

*/

}


function update_show_distance_division(){
	show_distance_division = !show_distance_division;
}

function update_distance_text_step(){
	distance_text_step = Number(this.value());
}

function update_show_text_for_distance_division() {
	show_text_for_distance_division = !show_text_for_distance_division
}

function update_show_angle_division(){
	show_angle_division = !show_angle_division;
}


function update_angle_major_division_step(){
	angle_major_division_step = Number(this.value());
}

function update_show_angle_major_division_step(){
	show_angle_major_division_step = !show_angle_major_division_step;
}

function update_angle_minor_division_step(){
	angle_minor_division_step = Number(this.value());
}

function update_show_angle_minor_division_step(){
	show_angle_minor_division_step = !show_angle_minor_division_step;
}

function update_angle_text_step(){
	angle_text_step = Number(this.value());
}

function update_show_angle_text_step(){
	show_angle_text_step =!show_angle_text_step;
}





function show_gyro_window(){
    var left_pos = window.innerWidth - 520;
    var top_pos = 10;
    window.open("http://localhost:8888/gyro.html",'_blank',"toolbar=no,location=no,status=no, menubar=no,titlebar=no, scrollbars=yes,resizable=yes,width=500,height=500, left="+left_pos+",top="+top_pos+" ");
}
function show_compass_window(){
    var left_pos1 = window.innerWidth - 520;
    var top_pos1 = window.innerHeight - 10;
    window.open("http://localhost:8888/compass.html",'_blank',"toolbar=no,location=no,status=no, menubar=no,titlebar=no, scrollbars=yes,resizable=yes,width=500,height=500, left="+left_pos1+",top="+top_pos1+" ");
}

function update_show_sensor_axis(){
  show_sensor_axis = !show_sensor_axis;
}

function update_show_sensor_coverage(){
  show_sensor_caverage = !show_sensor_caverage;
}

function update_circle_number(){
number_of_circles = Number(this.value());
}

function update_line_number(){
number_of_lines = Number(this.value());
}

function update_max_distance(){
  max_distance = Number(this.value());
}

function update_distance_between_sensor_1_2(){
  distance_between_sensor_1_2 = Number(this.value());
}

function update_sensor_angle(){
  sensor_angle = Number(this.value());
  $(document).ready(function(){
  $.ajax({
    url: "http://localhost:8888/api?id=2&value="+sensor_angle,
    dataType: 'json',
    success: function(data) {
      
    },
    error: function(err) {
      console.log(err);
    }
  });        
});

}

function update_servo_status(){
  var status;
  if(Number(this.value()) == 0){
    status = 'r';
    center_servo_status = 1;
    button_label_for_center_servo = "Stop";
    this.value(center_servo_status);
    this.html(button_label_for_center_servo);
    this.class('btn btn-danger');
  }else {
    status = 's';
    center_servo_status = 0;
    button_label_for_center_servo = "Start";
    this.value(Number(center_servo_status));
    this.html(button_label_for_center_servo);
    this.class('btn btn-success');   
  }

  $(document).ready(function(){
  $.ajax({
    url: "http://localhost:8888/api?id=1&value="+status,
    dataType: 'json',
    success: function(data) {
      
    },
    error: function(err) {
      console.log(err);

    }
  });        
});  

}

// Helper functions
function round_int(value, precision) {
    var multiplier = Math.pow(10, precision || 0);
    return Math.round(value * multiplier) / multiplier;
}

function getRndInteger(min, max) {
    return Math.floor(Math.random() * (max - min) ) + min;
}

function isJson(str) {
    try {
        JSON.parse(str);
    } catch (e) {
        return false;
    }
    return true;
}

function CircleClass(xx, yy, fc, aa, dd) {
  this.x = xx;
  this.y = yy;
  this.frameCount = fc;
  this.angle = aa;
  this.distance = dd;
  
  this.draw = function() {
    strokeWeight(5);
    stroke(235,244,247,95);
    fill(235,244,247,95);
    ellipse( this.x, this.y, 3, 3 );
    strokeWeight(0.5);
    line(0,0,this.x,this.y);
  }
} 