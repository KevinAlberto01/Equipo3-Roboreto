import processing.video.*;

Capture video;

color color1;
color color2;
color color3;
color color4;

int cont = 0;

void setup() {
  size(640, 360);
  video = new Capture(this, 640, 360);
  video.start();
  // Start off tracking for red
  color1 = color(255, 0, 0);
  color2 = color(0, 255, 0);
  color3 = color(0, 0, 225);
  color4 = color(255, 255, 0);
}

void captureEvent(Capture video) {
  // Read image from the camera
  video.read();
}

void draw() {
  video.loadPixels();
  image(video, 0, 0);

  // Before we begin searching, the "world record" for closest color is set to a high number that is easy for the first pixel to beat.
  float worldRecord1 = 500; 
  float worldRecord2 = 500; 
  float worldRecord3 = 500; 
  float worldRecord4 = 500; 
  
  
  // XY coordinate of closest color
  int closestX1 = 0;
  int closestY1 = 0;
  int closestX2 = 0;
  int closestY2 = 0;
  int closestX3 = 0;
  int closestY3 = 0;
  int closestX4 = 0;
  int closestY4 = 0;

  // Begin loop to walk through every pixel
  for (int x = 0; x < video.width; x++ ) {
    for (int y = 0; y < video.height; y++ ) {
      int loc = x + y * video.width;
      // What is current color
      color currentColor = video.pixels[loc];
      float r1 = red(currentColor);
      float g1 = green(currentColor);
      float b1 = blue(currentColor);
      float r2 = red(color1);
      float g2 = green(color1);
      float b2 = blue(color1);
      float r3 = red(color2);
      float g3 = green(color2);
      float b3 = blue(color2);
      float r4 = red(color3);
      float g4 = green(color3);
      float b4 = blue(color3);
      float r5 = red(color4);
      float g5 = green(color4);
      float b5 = blue(color4);

      // Using euclidean distance to compare colors
      float d1 = dist(r1, g1, b1, r2, g2, b2); // We are using the dist( ) function to compare the current color with the color we are tracking.
      float d2 = dist(r1, g1, b1, r3, g3, b3);
      float d3 = dist(r1, g1, b1, r4, g4, b4);
      float d4 = dist(r1, g1, b1, r5, g5, b5);

      // If current color is more similar to tracked color than
      // closest color, save current location and current difference
      if (d1 < worldRecord1) {
        worldRecord1 = d1;
        closestX1 = x;
        closestY1 = y;
      }
      if (d2 < worldRecord2) {
        worldRecord2 = d2;
        closestX2 = x;
        closestY2 = y;
      }
      if (d3 < worldRecord3) {
        worldRecord3 = d3;
        closestX3 = x;
        closestY3 = y;
      }
      if (d4 < worldRecord4) {
        worldRecord4 = d4;
        closestX4 = x;
        closestY4 = y;
      }
    }
  }

  // We only consider the color found if its color distance is less than 10. 
  // This threshold of 10 is arbitrary and you can adjust this number depending on how accurate you require the tracking to be.
  if (worldRecord2 < 10) { 
    // Draw a circle at the tracked pixel
    fill(color1);
    strokeWeight(4.0);
    stroke(0);
    ellipse(closestX1, closestY1, 16, 16);
  }
  if (worldRecord2 < 10) { 
    // Draw a circle at the tracked pixel
    fill(color2);
    strokeWeight(4.0);
    stroke(0);
    ellipse(closestX2, closestY2, 16, 16);
  }
  if (worldRecord3 < 10) { 
    // Draw a circle at the tracked pixel
    fill(color3);
    strokeWeight(4.0);
    stroke(0);
    ellipse(closestX3, closestY3, 16, 16);
  }
  if (worldRecord4 < 10) { 
    // Draw a circle at the tracked pixel
    fill(color4);
    strokeWeight(4.0);
    stroke(0);
    ellipse(closestX4, closestY4, 16, 16);
  }
}

void mousePressed() {
  // Save color where the mouse is clicked in trackColor variable
  switch(cont){
    case 0:
      int loc1 = mouseX + mouseY*video.width;
      color1 = video.pixels[loc1];
      cont += 1;
      break;
    case 1:
      int loc2 = mouseX + mouseY*video.width;
      color2 = video.pixels[loc2];
      cont += 1;
      break;
    case 2:
      int loc3 = mouseX + mouseY*video.width;
      color3 = video.pixels[loc3];
      cont += 1;
      break;
    case 3:
      int loc4 = mouseX + mouseY*video.width;
      color4 = video.pixels[loc4];
      cont = 0;
      break;
  }
}
