import java.util.ArrayList;

// --- Simulation constants ---------------------------------------------------
final int FIELD_SIZE = 400;
final int GRID_COLS = 80;
final int GRID_ROWS = 80;
final int FOOT_COUNT = 3;
final int PATCH_COUNT = 10;

final float PREDICTION_HORIZON = 0.6f;
final int PREDICTION_STEPS = 6;
final float POTENTIAL_DECAY = 2.5f;
final float NEGATIVE_LAMBDA = 1.0f;
final float MAX_DT = 0.05f;
final float EPS = 1e-4f;

PotentialField potentialField;
ArrayList<FootTrack> footTracks = new ArrayList<FootTrack>();
ArrayList<PotentialPatch> positivePatches = new ArrayList<PotentialPatch>();

float lastUpdateMillis;

int colorDanger;
int colorNeutral;
int colorAttractor;

void settings() {
  size(FIELD_SIZE, FIELD_SIZE);
  smooth(4);
}

void setup() {
  frameRate(60);
  ellipseMode(CENTER);
  rectMode(CORNER);
  potentialField = new PotentialField(GRID_COLS, GRID_ROWS, FIELD_SIZE);
  initFootTracks();
  initPatches();
  colorMode(RGB, 255);
  colorDanger = color(250, 80, 70);
  colorNeutral = color(30, 30, 40);
  colorAttractor = color(70, 210, 255);
  lastUpdateMillis = millis();
}

void draw() {
  float dt = computeDeltaTime();
  updateFootTracks(dt);
  updatePatches(dt);
  recomputeGlobalField();
  drawPotentialField();
  drawPositivePatches();
  drawFootTracks();
  drawHUD(dt);
}

// --- Simulation loop helpers -----------------------------------------------

void initFootTracks() {
  footTracks.clear();
  float margin = 70;
  for (int i = 0; i < FOOT_COUNT; i++) {
    PVector start = new PVector(random(margin, FIELD_SIZE - margin), random(margin, FIELD_SIZE - margin));
    footTracks.add(new FootTrack(start));
  }
}

void initPatches() {
  positivePatches.clear();
  for (int i = 0; i < PATCH_COUNT; i++) {
    positivePatches.add(new PotentialPatch());
  }
}

float computeDeltaTime() {
  float now = millis();
  float dt = (now - lastUpdateMillis) / 1000.0f;
  lastUpdateMillis = now;
  if (dt <= 0 || Float.isNaN(dt)) {
    dt = 1.0f / 60.0f;
  }
  return constrain(dt, 0.0f, MAX_DT);
}

void updateFootTracks(float dt) {
  for (FootTrack foot : footTracks) {
    foot.update(dt);
  }
}

void updatePatches(float dt) {
  for (PotentialPatch patch : positivePatches) {
    patch.update(dt);
  }
}

void recomputeGlobalField() {
  potentialField.recompute(footTracks, positivePatches);
}

// --- Rendering --------------------------------------------------------------

void drawPotentialField() {
  background(18);
  noStroke();
  float cellSize = potentialField.getCellSize();
  for (int col = 0; col < GRID_COLS; col++) {
    float x = col * cellSize;
    for (int row = 0; row < GRID_ROWS; row++) {
      float y = row * cellSize;
      float value = potentialField.getValue(col, row);
      fill(potentialToColor(value));
      rect(x, y, cellSize + 1, cellSize + 1);
    }
  }
}

color potentialToColor(float value) {
  float range = max(abs(potentialField.getMinValue()), abs(potentialField.getMaxValue()));
  if (range < 1e-3f) {
    return colorNeutral;
  }
  float normalized = value / range * 0.5f + 0.5f;
  normalized = constrain(normalized, 0, 1);
  if (normalized < 0.5f) {
    float amt = normalized / 0.5f;
    return lerpColor(colorDanger, colorNeutral, amt);
  } else {
    float amt = (normalized - 0.5f) / 0.5f;
    return lerpColor(colorNeutral, colorAttractor, amt);
  }
}

void drawFootTracks() {
  stroke(255, 130);
  strokeWeight(1.2f);
  noFill();
  for (FootTrack foot : footTracks) {
    beginShape();
    for (int i = 0; i <= PREDICTION_STEPS; i++) {
      float tau = (i / (float) PREDICTION_STEPS) * PREDICTION_HORIZON;
      PVector future = foot.predictPosition(tau);
      vertex(future.x, future.y);
    }
    endShape();
  }

  noStroke();
  for (FootTrack foot : footTracks) {
    fill(255, 230);
    ellipse(foot.position.x, foot.position.y, 10, 10);
    fill(255, 120);
    ellipse(foot.anchor.x, foot.anchor.y, 6, 6);
  }
}

void drawPositivePatches() {
  noFill();
  stroke(colorAttractor, 150);
  strokeWeight(1.0f);
  for (PotentialPatch patch : positivePatches) {
    float d = patch.radius * 2.0f;
    ellipse(patch.position.x, patch.position.y, d, d);
  }
}

void drawHUD(float dt) {
  fill(255);
  textSize(12);
  String info = "Δt " + nf(dt, 1, 3) + "s  |  " +
    "U range [" + nf(potentialField.getMinValue(), 1, 2) + ", " + nf(potentialField.getMaxValue(), 1, 2) + "]";
  text(info, 10, height - 12);
}

// --- Potential grid --------------------------------------------------------

class PotentialField {
  final int cols;
  final int rows;
  final float cellSize;
  float[][] values;
  float minValue = 0;
  float maxValue = 0;

  PotentialField(int cols, int rows, float fieldSize) {
    this.cols = cols;
    this.rows = rows;
    this.cellSize = fieldSize / (float) cols;
    values = new float[cols][rows];
  }

  void recompute(ArrayList<FootTrack> feet, ArrayList<PotentialPatch> patches) {
    minValue = Float.POSITIVE_INFINITY;
    maxValue = Float.NEGATIVE_INFINITY;
    for (int col = 0; col < cols; col++) {
      for (int row = 0; row < rows; row++) {
        float sampleX = (col + 0.5f) * cellSize;
        float sampleY = (row + 0.5f) * cellSize;
        PVector sample = new PVector(sampleX, sampleY);
        float value = 0;
        for (FootTrack foot : feet) {
          value += foot.potentialAt(sample);
        }
        for (PotentialPatch patch : patches) {
          value += patch.potentialAt(sample);
        }
        values[col][row] = value;
        if (value < minValue) {
          minValue = value;
        }
        if (value > maxValue) {
          maxValue = value;
        }
      }
    }
    if (!Float.isFinite(minValue) || !Float.isFinite(maxValue)) {
      minValue = 0;
      maxValue = 0;
    }
  }

  float getCellSize() {
    return cellSize;
  }

  float getValue(int col, int row) {
    return values[col][row];
  }

  float getMinValue() {
    return minValue;
  }

  float getMaxValue() {
    return maxValue;
  }

  float sample(float x, float y) {
    int col = constrain(floor(x / cellSize), 0, cols - 1);
    int row = constrain(floor(y / cellSize), 0, rows - 1);
    return values[col][row];
  }

  PVector sampleGradient(float x, float y) {
    int col = constrain(floor(x / cellSize), 0, cols - 1);
    int row = constrain(floor(y / cellSize), 0, rows - 1);
    int colL = max(0, col - 1);
    int colR = min(cols - 1, col + 1);
    int rowB = max(0, row - 1);
    int rowT = min(rows - 1, row + 1);
    float gradX = (values[colR][row] - values[colL][row]) / ((colR - colL) * cellSize + EPS);
    float gradY = (values[col][rowT] - values[col][rowB]) / ((rowT - rowB) * cellSize + EPS);
    return new PVector(gradX, gradY);
  }
}

// --- FootTrack state machine -----------------------------------------------

class FootTrack {
  static final int STATE_IDLE = 0;
  static final int STATE_MOVE_OUT = 1;
  static final int STATE_ROAM = 2;
  static final int STATE_RETURN = 3;

  PVector position;
  PVector velocity = new PVector();
  PVector anchor;
  PVector roamTarget;
  float stateTime = 0;
  float stateDuration = 1;
  float stateSpeed = 0;
  float orbitAngle = 0;
  float roamRadius = 45;
  float noiseCursor = 0;
  float jitterSeed;
  float wanderSeed;
  int state = STATE_IDLE;

  FootTrack(PVector start) {
    position = start.copy();
    anchor = start.copy();
    jitterSeed = random(5000);
    wanderSeed = random(8000, 12000);
    noiseCursor = random(1000);
    chooseRoamTarget();
    transitionToState(STATE_IDLE);
  }

  void update(float dt) {
    stateTime += dt;
    switch (state) {
    case STATE_IDLE:
      updateIdle(dt);
      break;
    case STATE_MOVE_OUT:
      updateMoveOut(dt);
      break;
    case STATE_ROAM:
      updateRoam(dt);
      break;
    case STATE_RETURN:
      updateReturn(dt);
      break;
    }
    clampToField();
    if (stateTime >= stateDuration) {
      advanceState();
    }
  }

  void updateIdle(float dt) {
    noiseCursor += dt * 0.6f;
    float jitterRange = 30;
    float jitterX = (noise(jitterSeed, noiseCursor) - 0.5f) * jitterRange * 2.0f;
    float jitterY = (noise(jitterSeed + 12.3f, noiseCursor * 1.1f) - 0.5f) * jitterRange * 2.0f;
    PVector jitterTarget = PVector.add(anchor, new PVector(jitterX, jitterY));
    moveTowards(jitterTarget, dt, 80);
  }

  void updateMoveOut(float dt) {
    moveTowards(roamTarget, dt, stateSpeed);
    if (position.dist(roamTarget) < 8) {
      transitionToState(STATE_ROAM);
    }
  }

  void updateRoam(float dt) {
    float angularSpeed = stateSpeed / max(20, roamRadius);
    orbitAngle += angularSpeed * dt;
    float wobble = (noise(wanderSeed, orbitAngle * 0.15f) - 0.5f) * 12;
    float dynamicRadius = constrain(roamRadius + wobble, 30, 65);
    PVector orbit = new PVector(cos(orbitAngle), sin(orbitAngle));
    orbit.mult(dynamicRadius);
    PVector orbitPoint = PVector.add(roamTarget, orbit);
    moveTowards(orbitPoint, dt, max(80, stateSpeed * 0.6f));
  }

  void updateReturn(float dt) {
    moveTowards(anchor, dt, stateSpeed);
    if (position.dist(anchor) < 6) {
      transitionToState(STATE_IDLE);
    }
  }

  void advanceState() {
    if (state == STATE_IDLE) {
      transitionToState(STATE_MOVE_OUT);
    } else if (state == STATE_MOVE_OUT) {
      transitionToState(STATE_ROAM);
    } else if (state == STATE_ROAM) {
      transitionToState(STATE_RETURN);
    } else {
      transitionToState(STATE_IDLE);
    }
  }

  void transitionToState(int nextState) {
    state = nextState;
    stateTime = 0;
    switch (state) {
    case STATE_IDLE:
      stateDuration = random(5.0f, 8.0f);
      stateSpeed = 30;
      break;
    case STATE_MOVE_OUT:
      chooseRoamTarget();
      stateDuration = random(1.0f, 2.0f);
      stateSpeed = random(120, 200);
      break;
    case STATE_ROAM:
      stateDuration = random(3.0f, 6.0f);
      stateSpeed = random(90, 140);
      roamRadius = random(35, 60);
      break;
    case STATE_RETURN:
      stateDuration = random(1.5f, 2.5f);
      stateSpeed = random(110, 180);
      break;
    }
  }

  void chooseRoamTarget() {
    float margin = 40;
    float x = random(margin, FIELD_SIZE - margin);
    float y = random(margin, FIELD_SIZE - margin);
    if (random(1) < 0.5f) {
      y = (random(1) < 0.5f) ? margin : FIELD_SIZE - margin;
    } else {
      x = (random(1) < 0.5f) ? margin : FIELD_SIZE - margin;
    }
    roamTarget = new PVector(x, y);
  }

  void moveTowards(PVector target, float dt, float speed) {
    PVector prev = position.copy();
    PVector desired = PVector.sub(target, position);
    float dist = desired.mag();
    if (dist < EPS) {
      velocity.mult(0.8f);
      return;
    }
    float maxStep = max(5, speed) * dt;
    if (maxStep < dist) {
      desired.mult(maxStep / dist);
    }
    position.add(desired);
    PVector delta = PVector.sub(position, prev);
    velocity = delta.copy().div(max(dt, EPS));
  }

  void clampToField() {
    position.x = constrain(position.x, 5, FIELD_SIZE - 5);
    position.y = constrain(position.y, 5, FIELD_SIZE - 5);
  }

  PVector predictPosition(float tau) {
    PVector future;
    if (state == STATE_ROAM) {
      float futureAngle = orbitAngle + (stateSpeed / max(20, roamRadius)) * tau;
      PVector orbit = new PVector(cos(futureAngle), sin(futureAngle));
      orbit.mult(roamRadius);
      future = PVector.add(roamTarget, orbit);
    } else if (state == STATE_IDLE) {
      float futurePhase = noiseCursor + tau * 0.8f;
      float jitterRange = 25;
      float jitterX = (noise(jitterSeed, futurePhase) - 0.5f) * jitterRange * 2.0f;
      float jitterY = (noise(jitterSeed + 12.3f, futurePhase * 1.1f) - 0.5f) * jitterRange * 2.0f;
      future = PVector.add(anchor, new PVector(jitterX, jitterY));
    } else {
      PVector effectiveVel = velocity.copy();
      if (state == STATE_MOVE_OUT || state == STATE_RETURN) {
        PVector target = state == STATE_MOVE_OUT ? roamTarget : anchor;
        PVector desired = PVector.sub(target, position);
        if (desired.magSq() > 1) {
          desired.normalize();
          desired.mult(stateSpeed);
          effectiveVel = PVector.lerp(effectiveVel, desired, 0.6f);
        }
      }
      future = PVector.add(position, PVector.mult(effectiveVel, tau));
    }
    future.x = constrain(future.x, 0, FIELD_SIZE);
    future.y = constrain(future.y, 0, FIELD_SIZE);
    return future;
  }

  float potentialAt(PVector sample) {
    float total = 0;
    PVector prevCenter = position.copy();
    PVector dir = headingDirection();
    for (int i = 0; i <= PREDICTION_STEPS; i++) {
      float tau = (i / (float) PREDICTION_STEPS) * PREDICTION_HORIZON;
      PVector mu = predictPosition(tau);
      if (i > 0) {
        PVector delta = PVector.sub(mu, prevCenter);
        if (delta.magSq() > 4) {
          dir = delta.copy().normalize();
        }
      }
      float sigmaAlong = computeSigmaAlong() * (1.0f + tau * 1.1f);
      float sigmaPerp = computeSigmaPerp() * (1.0f + tau * 0.4f);
      float gaussian = gaussianContribution(sample, mu, dir, sigmaAlong, sigmaPerp);
      float weight = exp(-POTENTIAL_DECAY * tau);
      total += -NEGATIVE_LAMBDA * weight * gaussian;
      prevCenter = mu;
    }
    return total;
  }

  PVector headingDirection() {
    if (velocity.magSq() < 1) {
      return new PVector(1, 0);
    }
    PVector dir = velocity.copy();
    dir.normalize();
    return dir;
  }

  float computeSigmaAlong() {
    float speed = velocity.mag();
    return lerp(30, 110, constrain(speed / 200.0f, 0, 1));
  }

  float computeSigmaPerp() {
    float speed = velocity.mag();
    return lerp(24, 55, constrain(speed / 200.0f, 0, 1));
  }

  float gaussianContribution(PVector sample, PVector center, PVector dir, float sigmaAlong, float sigmaPerp) {
    PVector rel = PVector.sub(sample, center);
    float along = rel.x * dir.x + rel.y * dir.y;
    PVector perpDir = new PVector(-dir.y, dir.x);
    float perp = rel.x * perpDir.x + rel.y * perpDir.y;
    float termAlong = (along * along) / (sigmaAlong * sigmaAlong + EPS);
    float termPerp = (perp * perp) / (sigmaPerp * sigmaPerp + EPS);
    float exponent = -0.5f * (termAlong + termPerp);
    return exp(exponent);
  }
}

// --- Positive potential patches --------------------------------------------

class PotentialPatch {
  PVector position;
  PVector velocity = new PVector();
  float radius;
  float peak;
  float wanderSeed;
  float maxSpeed;

  PotentialPatch() {
    float margin = 50;
    position = new PVector(random(margin, FIELD_SIZE - margin), random(margin, FIELD_SIZE - margin));
    radius = random(30, 60);
    peak = random(0.12f, 0.3f);
    wanderSeed = random(1000, 5000);
    maxSpeed = random(5, 15);
  }

  void update(float dt) {
    wanderSeed += dt * 0.2f;
    float heading = noise(wanderSeed) * TWO_PI * 2.0f;
    PVector desired = new PVector(cos(heading), sin(heading));
    desired.mult(maxSpeed);
    velocity.lerp(desired, 0.05f);
    position.add(PVector.mult(velocity, dt));
    keepInsideField();
  }

  void keepInsideField() {
    float margin = radius * 0.5f;
    if (position.x < margin || position.x > FIELD_SIZE - margin) {
      velocity.x *= -1;
      position.x = constrain(position.x, margin, FIELD_SIZE - margin);
    }
    if (position.y < margin || position.y > FIELD_SIZE - margin) {
      velocity.y *= -1;
      position.y = constrain(position.y, margin, FIELD_SIZE - margin);
    }
  }

  float potentialAt(PVector sample) {
    float sigma = max(12, radius * 0.6f);
    float distSq = PVector.sub(sample, position).magSq();
    float exponent = -0.5f * (distSq / (sigma * sigma));
    return peak * exp(exponent);
  }
}
