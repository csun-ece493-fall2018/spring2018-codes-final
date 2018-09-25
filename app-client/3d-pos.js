rotx = 0;
roty = 0;
function setup() {
    createCanvas(windowWidth, windowHeight, WEBGL);
}

function draw() {
    background(0);

    push();
    noFill();
    stroke(255);
    box(250, 250, 250);
    pop();

    push();
    noFill();
    stroke(98,245,31);
    rotateX(rotx);
    rotateY(roty);
    ellipse(0, 0, 200);
    ellipse(0, 0, 150);
    ellipse(0, 0, 100);
    pop();
    //rotateZ(frameCount * 0.01)

}

function mouseDragged() {
    rate = 0.01;
    rotx += (pmouseY-mouseY) * rate;
    roty += (mouseX-pmouseX) * rate;
}