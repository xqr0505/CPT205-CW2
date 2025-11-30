#define FREEGLUT_STATIC
#include <GL/freeglut.h>
#include <cmath> 
#include <iostream> 
#include <vector>
#include <random> 
#define M_PI 3.1415926535

// ==========================================================
// VECTOR MATH UTILITIES
// ==========================================================

struct vec3 { float x, y, z; };

// Basic vector operations
vec3 vec3_add(vec3 a, vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
vec3 vec3_sub(vec3 a, vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
vec3 vec3_scale(vec3 v, float s) { return { v.x * s, v.y * s, v.z * s }; }
float vec3_length(vec3 v) { return sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
vec3 vec3_normalize(vec3 v) {
    float len = vec3_length(v);
    if (len > 0) return vec3_scale(v, 1.0f / len);
    return { 0, 0, 0 };
}
vec3 vec3_lerp(vec3 a, vec3 b, float t) {
    return vec3_add(vec3_scale(a, 1.0f - t), vec3_scale(b, t));
}

vec3 vec3_create(float x, float y, float z) { return { x, y, z }; }

// ==========================================================
// CONSTANTS
// ==========================================================

// Floating disc constants
const float CENTRAL_DISC_RADIUS = 7.0f;
const float CENTRAL_DISC_HEIGHT = 0.5f;
const float CENTRAL_DISC_Y_POS = 0.0f;
const float SURROUND_DISC_RADIUS = 3.0f;
const float SURROUND_DISC_HEIGHT = 0.3f;
const float SURROUND_DISC_DISTANCE = 12.0f;
const float surround_heights[] = { 1.0f, -0.5f, 2.5f, -1.5f, 0.7f };

// Robot constants
const float ROBOT_CHASSIS_WIDTH = 0.8f;    
const float ROBOT_CHASSIS_HEIGHT = 0.4f;   
const float ROBOT_CHASSIS_DEPTH = 1.2f;   
const float ROBOT_WHEEL_RADIUS = 0.3f;     
const float ROBOT_WHEEL_WIDTH = 0.15f;    
const float ROBOT_CAMERA_RADIUS = 0.25f;   
const float ROBOT_CAMERA_Y_OFFSET = 0.3f;  

const float ROBOT_MOVE_SPEED = 0.1f;
const float ROBOT_ROTATE_SPEED = 3.0f;

// Robotic arm constants
const float ARM_BASE_HEIGHT = 0.5f;
const float ARM_BASE_RADIUS = 0.4f;
const float ARM_JOINT_RADIUS = 0.15f;
const float ARM_SEGMENT_LENGTH = 2.0f;
const float ARM_SEGMENT_WIDTH = 0.2f;
const float NOZZLE_RADIUS = 0.1f;
const float NOZZLE_LENGTH = 0.2f;

// Skimmer aircraft constants
const float WING_SPAN = 1.2f;
const float WING_ROOT_CHORD = 0.8f;
const float WING_TIP_CHORD = 0.4f;
const float WING_THICKNESS = 0.08f;
const float STRIPE_THICKNESS = 0.04f;
const float SKIMMER_LENGTH = 1.5f;
const float SKIMMER_WIDTH = 0.5f;
const float SKIMMER1_SPEED = 0.8f;
const float SKIMMER2_SPEED = 0.65f;

// Particle system constants
const int MAX_PARTICLES = 1000;
const float GRAVITY = 9.8f;

// Building constants
const float BUILDING_HEIGHT = 6.5f;
const float BUILDING_BASE = 3.5f;

// Fractal Tree constants
const int FRACTAL_TREE_ITERATIONS = 3;       
const float TREE_INITIAL_HEIGHT = 1.5f;       
const float TREE_INITIAL_RADIUS = 0.1f;       
const float TREE_HEIGHT_DECAY = 0.7f;         
const float TREE_RADIUS_DECAY = 0.65f;        
const float TREE_LEAF_SIZE = 0.4f;            
const float TREE_BRANCH_ANGLE = 25.0f;        

// Stone path constants
const float STONE_WIDTH = 1.3f;      
const float STONE_DEPTH = 0.6f;      
const float STONE_HEIGHT = 0.05f;    
const float GAP = 0.2f;             
// ==========================================================
// DATA STRUCTURES
// ==========================================================

// Robot state
struct Robot {
    float posX, posY, posZ;
    float angleY;
    float wheelRotation;
};

// Particle for water effect
struct Particle {
    bool active;
    float life;
    float x, y, z;
    float vx, vy, vz;
};

enum MenuOption {
    MENU_TOGGLE_DAY_NIGHT = 1,
    MENU_TOGGLE_PATH,
    MENU_COLOR_CYAN,
    MENU_COLOR_ORANGE,
    MENU_COLOR_PURPLE,
    MENU_COLOR_GREEN,
    MENU_COLOR_YELLO,
    MENU_TOGGLE_INSTRUCTIONS
};


const float g_glowColors[5][3] = {
    {0.5f, 0.8f, 1.0f},      // blue
    {1.0f, 0.549f, 0.0f},    // orange
    {0.753f, 0.376f, 0.898f},// purple
    {0.0f, 1.0f, 0.502f},    // green 
    {1.0f, 1.0f, 0.0f}       // yellow
};

int g_currentGlowColorIndex = 0; 
const int TOTAL_COLORS = 5;


// ==========================================================
// GLOBAL STATE
// =========================================================

// Window and camera state
int g_windowWidth = 800;
int g_windowHeight = 600;
float g_cameraAngleY = 0.0f;
float g_cameraAngleX = 0.0f;
float g_zoomFactor = 1.0f;
bool g_mouseLeftDown = false;
int g_mouseX, g_mouseY;
bool g_isRobotView = false;

// Camera transition state
bool g_cameraTransitioning = false;
float g_cameraTransitionProgress = 0.0f;
const float CAMERA_TRANSITION_DURATION = 1.0f; 

// Store camera states for interpolation
struct CameraState {
    vec3 eye;
    vec3 lookAt;
    vec3 up;
};

CameraState g_cameraStartState;
CameraState g_cameraEndState;
CameraState g_currentCameraState;

// Robot state
Robot g_robot;
//float g_robotCameraAngleY = 0.0f;
bool g_robotLightOn = true;         
float g_robotLightBrightness = 1.0f;   

// Global lighting state
bool g_envLightOn = true;

// Robotic arm state
float armBaseAngle = 0.0f;
float armLowerAngle = 120.0f;
float armUpperAngle = 30.0f;
float nozzleAngle = 90.0f;
bool isWatering = false;

// Skimmer aircraft state
float g_skimmer1_progress = 0.0f;
float g_skimmer2_progress = 0.0f;
bool g_showFlightPath = false;
std::vector<vec3> g_skimmerPath1;
std::vector<vec3> g_skimmerPath2;

// Particle system
Particle waterParticles[MAX_PARTICLES];

// Fractal Tree state
std::string g_fractalTreeGrammar; 

// Texture IDs
GLuint g_texTreeBark = 0;
GLuint g_texTreeLeaf = 0;
GLuint g_texGroundCenter = 0;
GLuint g_texGroundSurround = 0;
GLuint g_texBush = 0;

// Sky Dome State
GLuint g_texSkyDay = 0;
GLuint g_texSkyNight = 0;
int g_currentSkyIndex = 0; // 0 = Day, 1 = Night

// Instruction Panel State
bool g_showInstructionPanel = false;
GLuint g_texControls = 0;


// ==========================================================
// FUNCTION DECLARATIONS
// ==========================================================
void initTextures();
// Initialization
void initGL();
void initPaths();
void initParticles();
void initRobot();
// Main loop callbacks
void display();
void reshape(int width, int height);
void keyboard(unsigned char key, int x, int y);
void keyboardUp(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void idle();

// Material functions
void setMaterial(const GLfloat* ambient, const GLfloat* diffuse, const GLfloat* specular, float shininess, const GLfloat* emission);
void setBuildingFrameMaterial();
void setGlowingMaterial(const GLfloat* emissionColor);

// Primitive drawing functions
void drawFloatingDisc(float radius, float height);
void drawCube(float width, float height, float depth);
void drawSphere(float radius);

// Garden scene
void drawBush();
void drawGardenScene();


// Robotic arm
void drawArmBase();
void drawArmJoint();
void drawArmSegment();
void drawNozzle();
void drawWateringArm();
void calculateNozzleWorldPosition(float baseRot, float lowerArmRot, float upperArmRot, GLdouble outPos[3]);

// Building
void drawBuilding(int type);

// Skimmer aircraft
void drawWing();
void drawGlowingRing(float z_position, float thickness);
void drawSkimmer();
vec3 getCatmullRomPoint(vec3 p0, vec3 p1, vec3 p2, vec3 p3, float t);
vec3 getPointOnPath(float progress, const std::vector<vec3>& path);

// Robot
void drawRobot();
void updateRobotMovement(unsigned char key);
bool checkRobotBoundary(float x, float z);

// Particle system
void updateParticles(float dt);
void drawWaterParticles();

// Camera
void setupCamera();
void drawInstructionPanel();
// ==========================================================
// TEXTURE LOADING FUNCTIONS
// ==========================================================

// Load BMP texture from file
GLuint loadTexture(const char* filename) {
    GLuint textureID = 0;
    FILE* file;

    if (fopen_s(&file, filename, "rb") != 0 || file == NULL) {
        std::cerr << "[Texture Error] Failed to open file: " << filename << std::endl;
        return 0;
    }

    unsigned char header[54];
    if (fread(header, 1, 54, file) != 54) {
        std::cerr << "[Texture Error] Invalid BMP file: " << filename << std::endl;
        fclose(file);
        return 0;
    }

    if (header[0] != 'B' || header[1] != 'M') {
        std::cerr << "[Texture Error] Not a BMP file: " << filename << std::endl;
        fclose(file);
        return 0;
    }

    // Bit Count
    unsigned short bitCount = *(unsigned short*)&(header[0x1C]);

    int width = *(int*)&(header[0x12]);
    int height = *(int*)&(header[0x16]);
    unsigned int dataPos = *(unsigned int*)&(header[0x0A]);
    if (dataPos == 0) dataPos = 54; 

    //3 for RGB, 4 for RGBA
    int bytesPerPixel = 0;
    GLenum format; 

    if (bitCount == 24) {
        bytesPerPixel = 3;
        format = GL_BGR_EXT; 
    }
    else if (bitCount == 32) {
        bytesPerPixel = 4;
        format = GL_BGRA_EXT;
    }
    else {
        std::cerr << "[Texture Error] Unsupported bit depth: " << bitCount << " (Only 24 or 32 supported)" << std::endl;
        fclose(file);
        return 0;
    }

    int imageSize = width * height * bytesPerPixel;
    int rowSizePadded = (width * bytesPerPixel + 3) & (~3);
    int padding = rowSizePadded - (width * bytesPerPixel);

    unsigned char* data = new unsigned char[imageSize];
    unsigned char* ptr = data; 

    fseek(file, dataPos, SEEK_SET);

    for (int i = 0; i < height; i++) {
        fread(ptr, 1, width * bytesPerPixel, file);
        ptr += width * bytesPerPixel;

        if (padding > 0) {
            fseek(file, padding, SEEK_CUR);
        }
    }

    fclose(file);


    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1); 

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

    gluBuild2DMipmaps(GL_TEXTURE_2D, bytesPerPixel, width, height, format, GL_UNSIGNED_BYTE, data);

    delete[] data;

    std::cout << "[Texture Success] Loaded: " << filename
        << " (" << width << "x" << height << ", " << bitCount << "-bit)"
        << " ID: " << textureID << std::endl;

    return textureID;
}

// ==========================================================
// ROBOT LIGHTING CALCULATION FUNCTIONS
// ==========================================================

/**
 * @brief Calculate the world position of the robot's headlight.
 */
void calculateRobotLightWorldPosition(GLdouble outPos[3]) {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Navigate to robot position
    float robotGroundY = (CENTRAL_DISC_HEIGHT / 2.0f) + ROBOT_WHEEL_RADIUS;
    glTranslatef(g_robot.posX, robotGroundY, g_robot.posZ);
    glRotatef(g_robot.angleY, 0.0f, 1.0f, 0.0f);

    // Navigate to camera head position
    glTranslatef(0.0f, ROBOT_CHASSIS_HEIGHT / 2.0f + ROBOT_CAMERA_Y_OFFSET, 0.0f);
    glTranslatef(0.0f, 0.0f, ROBOT_CAMERA_RADIUS);

    GLdouble matrix[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, matrix);
    outPos[0] = matrix[12];
    outPos[1] = matrix[13];
    outPos[2] = matrix[14];

    glPopMatrix();
}

/**
 * @brief Calculate the direction vector of the robot's spotlight.
 * robot's forward direction
 */
void calculateRobotLightWorldDirection(GLdouble outDir[3]) {
    float angleRad = g_robot.angleY * M_PI / 180.0f;

    outDir[0] = sin(angleRad);
    outDir[1] = 0.0f;  
    outDir[2] = cos(angleRad);
}

/**
 * @brief Setup all lights.
 */
void setupLights() {
    // --- Setup LIGHT0 (ambient light) ---
    if (g_envLightOn) {
        glEnable(GL_LIGHT0);
        GLfloat light0_ambient[] = { 0.15f, 0.15f, 0.2f, 1.0f };
        GLfloat light0_diffuse[] = { 1.0f, 1.0f, 0.95f, 1.0f };
        GLfloat light0_position[] = { 0.4f, 1.0f, -1.0f, 0.0f };
        glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
        glLightfv(GL_LIGHT0, GL_SPECULAR, light0_ambient);
        glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
    }
    else {
        glDisable(GL_LIGHT0);
    }


    // --- Setup LIGHT1 (Robot's spotlight) ---
    if (g_robotLightOn) {
        glEnable(GL_LIGHT1);

        // world position
        float robotGroundY = (CENTRAL_DISC_HEIGHT / 2.0f) + ROBOT_WHEEL_RADIUS;
        float angleRad = g_robot.angleY * M_PI / 180.0f;

        // Light position: at the camera head
        GLfloat lightPosX = g_robot.posX + sin(angleRad) * ROBOT_CAMERA_RADIUS;
        GLfloat lightPosY = robotGroundY + (ROBOT_CHASSIS_HEIGHT / 2.0f) + ROBOT_CAMERA_Y_OFFSET;
        GLfloat lightPosZ = g_robot.posZ + cos(angleRad) * ROBOT_CAMERA_RADIUS;

        GLfloat light1_position[] = { lightPosX, lightPosY, lightPosZ, 1.0f };

        // Light direction: forward direction of the robot
        GLfloat light1_direction[] = {
            sin(angleRad),
            0.0f,  
            cos(angleRad)
        };

        // Set light properties
        glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
        glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light1_direction);
        glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 35.0f);      
        glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 15.0f);    

        // Set light attenuation
        glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0.5f);
        glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.08f);
        glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.01f);

        // light color
        GLfloat light1_diffuse[] = { 1.0f, 0.9f, 0.7f, 1.0f };
        light1_diffuse[0] *= g_robotLightBrightness;
        light1_diffuse[1] *= g_robotLightBrightness;
        light1_diffuse[2] *= g_robotLightBrightness;

        GLfloat light1_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
        GLfloat light1_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
        light1_specular[0] *= g_robotLightBrightness;
        light1_specular[1] *= g_robotLightBrightness;
        light1_specular[2] *= g_robotLightBrightness;

        glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
        glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
        glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
    }
    else {
        glDisable(GL_LIGHT1);
    }
}

// ==========================================================
// MATERIAL FUNCTIONS
// ==========================================================

/**
 * @brief Set material properties
 */
void setMaterial(const GLfloat* ambient, const GLfloat* diffuse, const GLfloat* specular, float shininess, const GLfloat* emission) {
    glDisable(GL_COLOR_MATERIAL);
    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);
}

/**
 * @brief  building material.
 */
void setBuildingFrameMaterial() {
    GLfloat ambient[] = { 0.15f, 0.15f, 0.2f, 1.0f };
    GLfloat diffuse[] = { 0.2f, 0.2f, 0.25f, 1.0f };
    GLfloat specular[] = { 0.4f, 0.4f, 0.5f, 1.0f };
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(ambient, diffuse, specular, 30.0f, emission);
}

/**
 * @brief glowing material.
 */
void setGlowingMaterial(const GLfloat* emissionColor) {
    GLfloat black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(black, black, black, 0.0f, emissionColor);
}

/**
 * @brief material properties.
 */
void setSkimmerBodyMaterial() {
    GLfloat ambient[] = { 0.1f, 0.1f, 0.15f, 1.0f };
    GLfloat diffuse[] = { 0.85f, 0.85f, 0.95f, 1.0f };
    GLfloat specular[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(ambient, diffuse, specular, 3.0f, emission);
}

/**
 * @brief tree trunks material.
 */
void setTreeTrunkMaterial() {
    GLfloat ambient[] = { 0.4f, 0.25f, 0.15f, 1.0f };
    GLfloat diffuse[] = { 0.5f, 0.35f, 0.2f, 1.0f };
    GLfloat specular[] = { 0.1f, 0.05f, 0.0f, 1.0f };
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(ambient, diffuse, specular, 5.0f, emission);
}

/**
 * @brief tree leaves material.
 */
void setTreeLeafMaterial() {
    GLfloat ambient[] = { 0.1f, 0.3f, 0.1f, 1.0f };
    GLfloat diffuse[] = { 0.2f, 0.6f, 0.2f, 1.0f };
    GLfloat specular[] = { 0.15f, 0.25f, 0.15f, 1.0f };
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(ambient, diffuse, specular, 20.0f, emission);
}

// Reset to default material
void resetMaterial() {
    glEnable(GL_COLOR_MATERIAL);
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);
}

// ==========================================================
// PRIMITIVE DRAWING FUNCTIONS
// ==========================================================

// Draw a floating disc 
void drawFloatingDisc(float radius, float height, GLuint textureID) {
    GLUquadric* quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);

    gluQuadricTexture(quadric, GL_TRUE);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // set color if texture load error
    if (textureID == 0) glDisable(GL_TEXTURE_2D);
    else glColor3f(1.0f, 1.0f, 1.0f);

    glPushMatrix();
    glTranslatef(0.0f, -height / 2.0f, 0.0f);
    glRotatef(90.0f, -1.0f, 0.0f, 0.0f);

    gluDisk(quadric, 0, radius, 80, 30);
    gluCylinder(quadric, radius, radius, height, 80, 10);

    glTranslatef(0.0f, 0.0f, height);
    gluDisk(quadric, 0, radius, 80, 30);

    glPopMatrix();

    glDisable(GL_TEXTURE_2D);
    gluDeleteQuadric(quadric);
}

// Draw a scaled cube
void drawCube(float width, float height, float depth) {
    glPushMatrix();
    glScalef(width, height, depth);
    glutSolidCube(1.0f);
    glPopMatrix();
}

// Draw a sphere
void drawSphere(float radius) {
    glutSolidSphere(radius, 20, 20);
}

// ==========================================================
// SKY DOME FUNCTIONS
// ==========================================================
void drawSkyDome() {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glTranslatef(0.0f, 0.0f, 0.0f);
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);

    glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT);

    glDisable(GL_LIGHTING);
    glDepthMask(GL_FALSE);

    glColor3f(1.0f, 1.0f, 1.0f);

    glEnable(GL_TEXTURE_2D);
    GLuint currentSky = (g_currentSkyIndex == 0) ? g_texSkyDay : g_texSkyNight;
    glBindTexture(GL_TEXTURE_2D, currentSky);

    if (currentSky == 0) {
        glDisable(GL_TEXTURE_2D);
        if (g_currentSkyIndex == 0) glColor3f(0.5f, 0.7f, 1.0f);
        else glColor3f(0.0f, 0.0f, 0.2f);
    }

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    GLUquadric* quad = gluNewQuadric();
    if (quad) {
        gluQuadricTexture(quad, GL_TRUE);
        gluQuadricOrientation(quad, GLU_INSIDE);
        float skyRadius = SURROUND_DISC_DISTANCE * 4.0f;
        gluSphere(quad, skyRadius, 50, 50);

        gluDeleteQuadric(quad);
    }

    glPopAttrib();
    glPopMatrix();
}

// ==========================================================
// GARDEN SCENE FUNCTIONS
// ==========================================================
/**
 * @brief Generates the instruction string for an L-System fractal tree.
 * F: Move forward and draw trunk/leaves
 * [: Save current state (position and orientation)
 * ]: Restore previous state
 * +: Rotate positively around X-axis (pitch up)
 * -: Rotate negatively around X-axis (pitch down)
 * &: Rotate positively around Y-axis (yaw left)
 * ^: Rotate negatively around Y-axis (yaw right)
 * /: Rotate positively around Z-axis (roll left)
 * \: Rotate negatively around Z-axis (roll right)
 */
void generateFractalTreeGrammar() {
    std::string axiom = "F"; // axiom: trunk
    std::string rule = "F[+F&F][-F^F][/F\F]";

    std::string currentString = axiom;

    for (int i = 0; i < FRACTAL_TREE_ITERATIONS; ++i) {
        std::string nextString = "";
        for (char c : currentString) {
            if (c == 'F') {
                nextString += rule; // apply rules
            }
            else {
                nextString += c; 
            }
        }
        currentString = nextString;
    }
    g_fractalTreeGrammar = currentString;
 
}

/**
 * @brief Renders a single tree leaf
 * @param size The size of the leaf
 */
void drawTreeLeaf(float size) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, g_texTreeLeaf);

    if (g_texTreeLeaf != 0) glColor3f(1.0f, 1.0f, 1.0f);
    else glDisable(GL_TEXTURE_2D);

    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);

    glTexCoord2f(0.5f, 0.0f); glVertex3f(0, 0, 0);             
    glTexCoord2f(1.0f, 0.5f); glVertex3f(size / 2.0f, size, 0); 
    glTexCoord2f(0.5f, 1.0f); glVertex3f(0, size * 2.0f, 0);   
    glTexCoord2f(0.0f, 0.5f); glVertex3f(-size / 2.0f, size, 0);
    glEnd();

    glDisable(GL_TEXTURE_2D);
}

/**
 * @brief Renders a single tree branch segment (cylinder)
 * @param radius Base 
 * @param height
 */
void drawTreeBranch(float radius, float height) {
    glPushMatrix();
    // Rotate to align cylinder along Y-axis
    glRotatef(-90, 1.0f, 0.0f, 0.0f);

    GLUquadric* quad = gluNewQuadric();
    gluQuadricTexture(quad, GL_TRUE); 

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, g_texTreeBark);

    if (g_texTreeBark != 0) glColor3f(1.0f, 1.0f, 1.0f);
    else glDisable(GL_TEXTURE_2D);

    gluCylinder(quad, radius, radius * 0.8f, height, 8, 1);
    glDisable(GL_TEXTURE_2D);
    gluDeleteQuadric(quad);
    glPopMatrix();
}

/**
 * @brief Recursively renders a fractal tree based on L-System generated string
 */
void drawFractalTree() {
    if (g_fractalTreeGrammar.empty()) return;
    
    // Fixed seed for consistent random leaf orientations
    srand(0);
    
    float currentHeight = TREE_INITIAL_HEIGHT;
    float currentRadius = TREE_INITIAL_RADIUS;

    // Process each character in the grammar string
    for (char c : g_fractalTreeGrammar) {
        switch (c) {
        case 'F': // Draw forward and add leaf at end
            setTreeTrunkMaterial();
            drawTreeBranch(currentRadius, currentHeight);
            glTranslatef(0.0f, currentHeight, 0.0f);

            setTreeLeafMaterial();
            glPushMatrix();
            glRotatef(rand() % 360, 0.0f, 1.0f, 0.0f);        
            glRotatef((rand() % 40) - 20, 1.0f, 0.0f, 0.0f);  
            drawTreeLeaf(TREE_LEAF_SIZE);
            glPopMatrix();
            break;

        case '[': // Push current state (start branch)
            glPushMatrix();
            currentHeight *= TREE_HEIGHT_DECAY;
            currentRadius *= TREE_RADIUS_DECAY;
            break;

        case ']': // Pop state (return to parent branch)
            glPopMatrix();
            currentHeight /= TREE_HEIGHT_DECAY;
            currentRadius /= TREE_RADIUS_DECAY;
            break;

            // --- Rotation commands ---
        case '+': glRotatef(TREE_BRANCH_ANGLE, 1.0f, 0.0f, 0.0f); break;  // Pitch up
        case '-': glRotatef(-TREE_BRANCH_ANGLE, 1.0f, 0.0f, 0.0f); break; // Pitch down
        case '&': glRotatef(TREE_BRANCH_ANGLE, 0.0f, 1.0f, 0.0f); break;  // Yaw left
        case '^': glRotatef(-TREE_BRANCH_ANGLE, 0.0f, 1.0f, 0.0f); break; // Yaw right
        case '/': glRotatef(TREE_BRANCH_ANGLE, 0.0f, 0.0f, 1.0f); break;  // Roll clockwise
        case '\\': glRotatef(-TREE_BRANCH_ANGLE, 0.0f, 0.0f, 1.0f); break;// Roll counter-clockwise
            
        default:
            break; 
        }
    }
    resetMaterial();
}

// Draw bush
void drawBush() {
    const float t = (1.0f + sqrt(5.0f)) / 2.0f;
    const float r = 1.0f;
    const float scale = r / sqrt(1.0f * 1.0f + t * t);
    const float v1 = 1.0f * scale;
    const float v2 = t * scale;

    static const GLfloat vertices[12][3] = {
        { -v1, v2, 0 },{ v1, v2, 0 },{ -v1, -v2, 0 },{ v1, -v2, 0 },
        { 0, -v1, v2 },{ 0, v1, v2 },{ 0, -v1, -v2 },{ 0, v1, -v2 },
        { v2, 0, -v1 },{ v2, 0, v1 },{ -v2, 0, -v1 },{ -v2, 0, v1 }
    };

    static const GLint faces[20][3] = {
        { 0, 11, 5 },{ 0, 5, 1 },{ 0, 1, 7 },{ 0, 7, 10 },{ 0, 10, 11 },
        { 1, 5, 9 },{ 5, 11, 4 },{ 11, 10, 2 },{ 10, 7, 6 },{ 7, 1, 8 },
        { 3, 9, 4 },{ 3, 4, 2 },{ 3, 2, 6 },{ 3, 6, 8 },{ 3, 8, 9 },
        { 4, 9, 5 },{ 2, 4, 11 },{ 6, 2, 10 },{ 8, 6, 7 },{ 9, 8, 1 }
    };

    
    glEnable(GL_TEXTURE_2D);
	// Bind bush texture
    glBindTexture(GL_TEXTURE_2D, g_texBush);

    if (g_texBush != 0) glColor3f(1.0f, 1.0f, 1.0f);
    else glDisable(GL_TEXTURE_2D);

    glBegin(GL_TRIANGLES);
    for (int i = 0; i < 20; i++) {
        const GLfloat* ptr1 = vertices[faces[i][0]];
        const GLfloat* ptr2 = vertices[faces[i][1]];
        const GLfloat* ptr3 = vertices[faces[i][2]];
        glNormal3fv(ptr1);
        if (i % 2 == 0) glTexCoord2f(0.5f, 1.0f); 
        else            glTexCoord2f(0.0f, 0.0f); 
        glVertex3fv(ptr1);

        glNormal3fv(ptr2);
        if (i % 2 == 0) glTexCoord2f(0.0f, 0.0f); 
        else            glTexCoord2f(1.0f, 0.0f); 
        glVertex3fv(ptr2);

        glNormal3fv(ptr3);
        if (i % 2 == 0) glTexCoord2f(1.0f, 0.0f); 
        else            glTexCoord2f(0.5f, 1.0f); 
        glVertex3fv(ptr3);
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);
}



// Draw the garden scene with bushes
void drawGardenScene() {
    glColor3f(0.2f, 0.6f, 0.2f);

    // Enable clipping plane to cut bushes at ground level
    GLdouble planeEquation[4] = { 0.0, 1.0, 0.0, 0.0 };
    glClipPlane(GL_CLIP_PLANE0, planeEquation);
    glEnable(GL_CLIP_PLANE0);

    // Draw multiple bushes at different positions
    vec3 bushPositions[] = {
        {-3.0f, 0.0f, 3.0f},
        {-5.0f, 0.0f, 2.0f},
        {-3.8f, 0.0f, 1.3f},
        {-5.5f, 0.0f, 0.5f},
        {-4.8f, 0.0f, -1.6f}
    };
    vec3 bushScales[] = {
        {1.5f, 1.5f, 1.5f},
        {1.2f, 1.2f, 1.2f},
        {0.8f, 0.6f, 0.8f},
        {1.4f, 1.4f, 1.4f},
        {1.6f, 1.6f, 1.6f},
    };

    for (int i = 0; i < 5; i++) {
        glPushMatrix();
        glTranslatef(bushPositions[i].x, bushPositions[i].y, bushPositions[i].z);
        glScalef(bushScales[i].x, bushScales[i].y, bushScales[i].z);
        drawBush();
        glPopMatrix();
    }

    glDisable(GL_CLIP_PLANE0);
    glPushMatrix();
    glTranslatef(2.5f, 0.0f, -4.0f);
    glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);

    drawFractalTree();
    glPopMatrix();
}



/**
 * draw the stone path
 */
void drawStonePath(float centerX, float centerZ, float radius, float startAngleDeg, float endAngleDeg, float stoneRotationDeg) {

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, g_texGroundSurround);

    if (g_texGroundSurround != 0) glColor3f(1.0f, 1.0f, 1.0f);
    else {
        glDisable(GL_TEXTURE_2D);
        glColor3f(0.5f, 0.5f, 0.55f);
    }

    float startRad = startAngleDeg * M_PI / 180.0f;
    float endRad = endAngleDeg * M_PI / 180.0f;
    float totalAngleRad = fabs(endRad - startRad);
    float arcLength = totalAngleRad * radius;
    float stepUnit = STONE_WIDTH + GAP;
    int numStones = (int)(arcLength / stepUnit);

    if (numStones <= 0) numStones = 1;

    for (int i = 0; i <= numStones; ++i) {
        float t = (float)i / numStones;
        float currentAngle = startRad + t * (endRad - startRad);
        float x = centerX + radius * sin(currentAngle);
        float z = centerZ + radius * cos(currentAngle);
        float distSq = x * x + z * z;
        float limitRadius = CENTRAL_DISC_RADIUS - 0.5f;
        float limitSq = limitRadius * limitRadius;

        if (distSq > limitSq) {
            continue;
        }

        glPushMatrix();
        {
            glTranslatef(x, 0.02f, z);
            glRotatef(stoneRotationDeg, 0.0f, 1.0f, 0.0f);

            float w = STONE_WIDTH / 2.0f;
            float h = STONE_HEIGHT;
            float d = STONE_DEPTH / 2.0f;

            glBegin(GL_QUADS);
            glNormal3f(0.0f, 1.0f, 0.0f);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(-w, h, -d);
            glTexCoord2f(0.0f, 1.0f); glVertex3f(-w, h, d);
            glTexCoord2f(1.0f, 1.0f); glVertex3f(w, h, d);
            glTexCoord2f(1.0f, 0.0f); glVertex3f(w, h, -d);
            glEnd();

            glDisable(GL_TEXTURE_2D);
            glColor3f(0.4f, 0.4f, 0.45f);
            glBegin(GL_QUAD_STRIP);
            glNormal3f(0, 0, -1); glVertex3f(w, 0, -d); glVertex3f(w, h, -d);
            glVertex3f(-w, 0, -d); glVertex3f(-w, h, -d);
            glNormal3f(-1, 0, 0); glVertex3f(-w, 0, d); glVertex3f(-w, h, d);
            glNormal3f(0, 0, 1);  glVertex3f(w, 0, d); glVertex3f(w, h, d);
            glNormal3f(1, 0, 0);  glVertex3f(w, 0, -d); glVertex3f(w, h, -d);
            glEnd();

            if (g_texGroundSurround != 0) {
                glEnable(GL_TEXTURE_2D);
                glColor3f(1.0f, 1.0f, 1.0f);
            }
        }
        glPopMatrix();
    }

    glDisable(GL_TEXTURE_2D);
}
// ==========================================================
// ROBOTIC ARM FUNCTIONS
// ==========================================================

void drawArmBase() {
    glPushMatrix();
    glColor3f(0.2f, 0.2f, 0.2f);
    drawCube(ARM_BASE_RADIUS, ARM_BASE_HEIGHT, ARM_BASE_RADIUS);
    glPopMatrix();
}

void drawArmJoint() {
    glColor3f(0.2f, 0.2f, 0.2f);
    drawSphere(ARM_JOINT_RADIUS);
}

void drawArmSegment() {
    glPushMatrix();
    glColor3f(0.75f, 0.75f, 0.75f);
    drawCube(ARM_SEGMENT_LENGTH, ARM_SEGMENT_WIDTH, ARM_SEGMENT_WIDTH);
    glPopMatrix();
}

void drawNozzle() {
    glPushMatrix();
    glColor3f(0.75f, 0.75f, 0.75f);
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, NOZZLE_RADIUS, NOZZLE_RADIUS, NOZZLE_LENGTH, 20, 20);
    gluDeleteQuadric(quad);
    glPopMatrix();
}

// Draw complete robotic arm with hierarchical transformations
void drawWateringArm() {
    glPushMatrix();
    {
        glTranslatef(0.0f, 0.1f, 0.0f);
        glRotatef(armBaseAngle, 0.0f, 1.0f, 0.0f);
        drawArmBase();

        glPushMatrix();
        {
            glTranslatef(0.0f, ARM_BASE_HEIGHT, 0.0f);
            glRotatef(armLowerAngle, 0.0f, 0.0f, 1.0f);
            drawArmJoint();

            glPushMatrix();
            glTranslatef(ARM_SEGMENT_LENGTH / 2.0f, 0.0f, 0.0f);
            drawArmSegment();
            glPopMatrix();

            glTranslatef(ARM_SEGMENT_LENGTH, 0.0f, 0.0f);
            glRotatef(armUpperAngle, 0.0f, 0.0f, 1.0f);
            drawArmJoint();

            glPushMatrix();
            glTranslatef(ARM_SEGMENT_LENGTH / 2.0f, 0.0f, 0.0f);
            drawArmSegment();
            glPopMatrix();

            glTranslatef(ARM_SEGMENT_LENGTH, 0.0f, 0.0f);
            glRotatef(nozzleAngle, 0.0f, 0.0f, 1.0f);
            drawNozzle();
        }
        glPopMatrix();
    }
    glPopMatrix();
}

// Calculate nozzle world position for particle emission
void calculateNozzleWorldPosition(float baseRot, float lowerArmRot, float upperArmRot, GLdouble outPos[3]) {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glTranslatef(0.0f, CENTRAL_DISC_Y_POS, 0.0f);
    glTranslatef(0.0f, CENTRAL_DISC_HEIGHT / 2.0f, 0.0f);
    glTranslatef(0.0f, 0.1f, 0.0f);
    glRotatef(baseRot, 0.0f, 1.0f, 0.0f);
    glTranslatef(0.0f, ARM_BASE_HEIGHT, 0.0f);
    glRotatef(lowerArmRot, 0.0f, 0.0f, 1.0f);
    glTranslatef(ARM_SEGMENT_LENGTH, 0.0f, 0.0f);
    glRotatef(upperArmRot, 0.0f, 0.0f, 1.0f);
    glTranslatef(ARM_SEGMENT_LENGTH, 0.0f, 0.0f);
    glRotatef(nozzleAngle, 0.0f, 0.0f, 1.0f);
    glTranslatef(NOZZLE_LENGTH, 0.0f, 0.0f);

    GLdouble matrix[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, matrix);
    outPos[0] = matrix[12];
    outPos[1] = matrix[13];
    outPos[2] = matrix[14];

    glPopMatrix();
}

// ==========================================================
// BUILDING FUNCTIONS 
// ==========================================================

/**
 * @brief draw a building of specified type
 */
void drawBuilding(int type) {
    float bWidth, bHeight; 
    int floors;           
    GLfloat r = g_glowColors[g_currentGlowColorIndex][0];
    GLfloat g = g_glowColors[g_currentGlowColorIndex][1];
    GLfloat b = g_glowColors[g_currentGlowColorIndex][2];

    GLfloat windowEmission[] = { r, g, b, 1.0f };

    switch (type) {
    case 0:
        bWidth = 3.5f; bHeight = 7.0f; floors = 12;
        break;
    case 1:
        bWidth = 4.0f; bHeight = 6.5f; floors = 8;
        break;
    case 2:
        bWidth = 3.7f; bHeight = 6.8f; floors = 6;
        break;
    case 3:
        bWidth = 3.2f; bHeight = 10.5f; floors = 8;
        break;
    case 4:
        bWidth = 3.8f; bHeight = 6.5f; floors = 5;
        break;
    default:
        bWidth = 3.5f; bHeight = 7.0f; floors = 10;
        break;
    }

    setBuildingFrameMaterial();
    glPushMatrix();
    glTranslatef(0.0f, bHeight / 2.0f, 0.0f);
    drawCube(bWidth, bHeight, bWidth);
    glPopMatrix();

    setGlowingMaterial(windowEmission);
    float depthOffset = bWidth * 0.505f;

    switch (type) {
    case 0:
    {
        float floorH = bHeight / floors;
        for (int i = 0; i < floors; ++i) {
            float y = i * floorH + floorH * 0.5f;
            for (int s = 0; s < 4; ++s) {
                glPushMatrix();
                glRotatef(90.0f * s, 0.0f, 1.0f, 0.0f);
                glTranslatef(0.0f, y, depthOffset);
                drawCube(bWidth * 0.8f, floorH * 0.6f, 0.05f);
                glPopMatrix();
            }
        }
    }
    break;

    case 1:
    {
        float floorH = bHeight / floors;
        for (int i = 0; i < floors; ++i) {
            float y = i * floorH + floorH * 0.5f;
            for (int s = 0; s < 4; ++s) {
                glPushMatrix();
                glRotatef(90.0f * s, 0.0f, 1.0f, 0.0f);
                glTranslatef(0.0f, y, depthOffset);
                drawCube(bWidth * 0.8f, floorH * 0.2f, 0.05f);
                glPopMatrix();
            }
        }
    }
    break;

    case 2:
    {
        float floorH = bHeight / floors;
        int winsPerFloor = 4;
        for (int i = 0; i < floors; ++i) {
            float y = i * floorH + floorH * 0.5f;

            for (int s = 0; s < 4; ++s) {
                glPushMatrix();
                glRotatef(90.0f * s, 0.0f, 1.0f, 0.0f);
                glTranslatef(0.0f, y, depthOffset);

                float rowW = bWidth * 0.9f;
                float step = rowW / winsPerFloor;
                float startX = -rowW / 2.0f + step / 2.0f;

                for (int w = 0; w < winsPerFloor; w++) {
                    glPushMatrix();
                    glTranslatef(startX + w * step, 0.0f, 0.0f);
                    drawCube(floorH * 0.5f, floorH * 0.35, 0.05f);
                    glPopMatrix();
                }
                glPopMatrix();
            }
        }
    }
    break;

    case 3:
    {
        int strips = 3;
        float stripW = bWidth / (strips * 2.0f);
        float startX = -(bWidth / 2.0f) + stripW;
        float gap = (bWidth - 2 * stripW) / (strips - 1);
        float stripH = bHeight * 0.9f;
        float y = bHeight / 2.0f;

        for (int s = 0; s < 4; ++s) {
            glPushMatrix();
            glRotatef(90.0f * s, 0.0f, 1.0f, 0.0f);
            glTranslatef(0.0f, y, depthOffset);

            for (int k = 0; k < strips; k++) {
                glPushMatrix();
                float x = -bWidth * 0.3f + k * (bWidth * 0.3f);
                glTranslatef(x, 0.0f, 0.0f);
                drawCube(bWidth * 0.06f, stripH, 0.05f);
                glPopMatrix();
            }
            glPopMatrix();
        }
    }
    break;

    case 4:
    {
        float lineThick = 0.25f;
        float halfW = bWidth / 2.0f;
        float halfH = bHeight / 2.0f;

        for (int s = 0; s < 4; ++s) {
            glPushMatrix();
            glRotatef(90.0f * s, 0.0f, 1.0f, 0.0f);

            glTranslatef(0.0f, halfH, depthOffset);


            glPushMatrix();
            glTranslatef(0.0f, halfH - lineThick / 2.0f, 0.0f);
            drawCube(bWidth, lineThick, 0.05f);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(0.0f, -(halfH - lineThick / 2.0f), 0.0f);
            drawCube(bWidth, lineThick, 0.05f);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(-(halfW - lineThick / 2.0f), 0.0f, 0.0f);
            drawCube(lineThick, bHeight - 2 * lineThick, 0.05f);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(halfW - lineThick / 2.0f, 0.0f, 0.0f);
            drawCube(lineThick, bHeight - 2 * lineThick, 0.05f);
            glPopMatrix();

            glPopMatrix();
        }
    }
    break;
    }

    resetMaterial();
}

// ==========================================================
// SKIMMER AIRCRAFT FUNCTIONS
// ==========================================================

// Draw aircraft wing
void drawWing() {
    glBegin(GL_QUADS);

    // Top surface
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);

    // Bottom surface
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);

    // Wing tip
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);

    // Trailing edge
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);

    // Leading edge
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);

    // Wing root
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);

    glEnd();
}

// Draw glowing decorative ring
void drawGlowingRing(float z_position, float thickness) {
    const int SEGMENTS = 24;
    const float RING_OFFSET = 0.03f;
    float z_ratio = z_position / SKIMMER_LENGTH;
    float fuselage_radius_at_z = SKIMMER_WIDTH * sqrt(1.0f - z_ratio * z_ratio);
    float ring_radius = fuselage_radius_at_z + RING_OFFSET;

    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= SEGMENTS; ++i) {
        float angle = 2.0f * M_PI * (float)i / SEGMENTS;
        float x = cos(angle);
        float y = sin(angle);
        vec3 normal = vec3_normalize(vec3_create(x, y, 0.0f));

        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(x * ring_radius, y * ring_radius, z_position - thickness / 2.0f);
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(x * ring_radius, y * ring_radius, z_position + thickness / 2.0f);
    }
    glEnd();
}

// Draw complete skimmer aircraft
void drawSkimmer() {
    //-- Fuselage --
    setSkimmerBodyMaterial();
    glPushMatrix();
    glScalef(SKIMMER_WIDTH, SKIMMER_WIDTH, SKIMMER_LENGTH);
    glutSolidSphere(1.0, 16, 12);
    glPopMatrix();

    //-- Wings --
    // Right wing
    glPushMatrix();
    glTranslatef(SKIMMER_WIDTH * 0.5f, 0.0f, 0.0f);
    glRotatef(5.0f, 0.0f, 0.0f, 1.0f);
    drawWing();
    glPopMatrix();

    // Left wing
    glPushMatrix();
    glTranslatef(-SKIMMER_WIDTH * 0.5f, 0.0f, 0.0f);
    glRotatef(5.0f, 0.0f, 0.0f, 1.0f);
    glScalef(-1.0f, 1.0f, 1.0f);
    glFrontFace(GL_CW);
    drawWing();
    glFrontFace(GL_CCW);
    glPopMatrix();

    //-- Glowing Rings ---
    GLfloat r = g_glowColors[g_currentGlowColorIndex][0];
    GLfloat g = g_glowColors[g_currentGlowColorIndex][1];
    GLfloat b = g_glowColors[g_currentGlowColorIndex][2];

    GLfloat ringEmission[] = { r, g, b, 1.0f };
    setGlowingMaterial(ringEmission);

    drawGlowingRing(SKIMMER_LENGTH * 0.3f, 0.2f);
    drawGlowingRing(SKIMMER_LENGTH * -0.3f, 0.2f);

    // Reset material state
    resetMaterial();
}
// Centripetal Catmull-Rom
vec3  getCatmullRomPoint(vec3 p0, vec3 p1, vec3 p2, vec3 p3, float t, float alpha = 0.5f) {
    auto tj = [alpha](float ti, vec3 pi, vec3 pj) {
        float dx = pj.x - pi.x;
        float dy = pj.y - pi.y;
        float dz = pj.z - pi.z;
        float dist = sqrt(dx * dx + dy * dy + dz * dz);
        return ti + pow(dist, alpha);
        };

    float t0 = 0.0f;
    float t1 = tj(t0, p0, p1);
    float t2 = tj(t1, p1, p2);
    float t3 = tj(t2, p2, p3);
    float tt = t1 + (t2 - t1) * t;

    vec3 A1 = vec3_add(vec3_scale(p0, (t1 - tt) / (t1 - t0)), vec3_scale(p1, (tt - t0) / (t1 - t0)));
    vec3 A2 = vec3_add(vec3_scale(p1, (t2 - tt) / (t2 - t1)), vec3_scale(p2, (tt - t1) / (t2 - t1)));
    vec3 A3 = vec3_add(vec3_scale(p2, (t3 - tt) / (t3 - t2)), vec3_scale(p3, (tt - t2) / (t3 - t2)));

    vec3 B1 = vec3_add(vec3_scale(A1, (t2 - tt) / (t2 - t0)), vec3_scale(A2, (tt - t0) / (t2 - t0)));
    vec3 B2 = vec3_add(vec3_scale(A2, (t3 - tt) / (t3 - t1)), vec3_scale(A3, (tt - t1) / (t3 - t1)));

    vec3 C = vec3_add(vec3_scale(B1, (t2 - tt) / (t2 - t1)), vec3_scale(B2, (tt - t1) / (t2 - t1)));
    return C;
}
// Get point on path with looping
vec3 getPointOnPath(float progress, const std::vector<vec3>& path) {
    if (path.size() < 4) return vec3_create(0, 0, 0);

    int numPoints = path.size();
    int p1_idx = (int)progress;
    float t = progress - p1_idx;

    int p0_idx = (p1_idx - 1 + numPoints) % numPoints;
    int p2_idx = (p1_idx + 1) % numPoints;
    int p3_idx = (p1_idx + 2) % numPoints;
    return getCatmullRomPoint(path[p0_idx], path[p1_idx], path[p2_idx], path[p3_idx], t, 0.5f);
}

// Initialize flight paths
void initPaths() {
    // Path 1
    g_skimmerPath1.push_back(vec3_create(20.0f, 8.0f, 0.0f));
    g_skimmerPath1.push_back(vec3_create(0.0f, 7.0f, 18.0f));
    g_skimmerPath1.push_back(vec3_create(-5.0f, 4.0f, 4.0f));
    g_skimmerPath1.push_back(vec3_create(-16.0f, 2.0f, 0.0f));
    g_skimmerPath1.push_back(vec3_create(-12.0f, 3.0f, -10.0f));
    g_skimmerPath1.push_back(vec3_create(0.0f, 9.0f, -12.0f));

    // Path 2
    g_skimmerPath2.push_back(vec3_create(5.0f, 7.0f, -4.0f));
    g_skimmerPath2.push_back(vec3_create(9.0f, 8.0f, 0.0f));
    g_skimmerPath2.push_back(vec3_create(2.0f, 5.0f, 9.0f));
    g_skimmerPath2.push_back(vec3_create(-12.0f, 3.0f, 16.0f));
    g_skimmerPath2.push_back(vec3_create(-14.0f, 6.0f, -10.0f));

}

// Draw skimmer flight path visualization
void drawFlightPath(const std::vector<vec3>& path, vec3 color) {
    if (path.empty()) return;

    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glColor3f(color.x, color.y, color.z);

    glBegin(GL_LINE_STRIP);
    for (float t = 0.0f; t < path.size(); t += 0.1f) {
        vec3 point = getPointOnPath(t, path);
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();

    glEnable(GL_LIGHTING);
    glLineWidth(1.0f);
}

// Draw animated skimmer
void drawAnimatedSkimmer(float progress, const std::vector<vec3>& path) {
    vec3 currentPos = getPointOnPath(progress, path);
    float next_progress = progress + 0.01f;
    if (next_progress >= path.size()) {
        next_progress -= path.size();
    }
    vec3 nextPos = getPointOnPath(next_progress, path);
    vec3 direction = vec3_normalize(vec3_sub(nextPos, currentPos));

    glPushMatrix();
    {
        glTranslatef(currentPos.x, currentPos.y, currentPos.z);
        float yaw = atan2(direction.x, direction.z) * 180.0 / M_PI;
        float pitch = asin(-direction.y) * 180.0 / M_PI;
        glRotatef(yaw, 0.0f, 1.0f, 0.0f);
        glRotatef(pitch, 1.0f, 0.0f, 0.0f);
        drawSkimmer();
    }
    glPopMatrix();
}

// ==========================================================
// ROBOT FUNCTIONS
// ==========================================================

// Check if robot position is within central disc boundary
bool checkRobotBoundary(float x, float z) {
    float distanceFromCenter = sqrt(x * x + z * z);
    return distanceFromCenter < CENTRAL_DISC_RADIUS - (ROBOT_CHASSIS_WIDTH / 2.0f);
}


void drawRobotChassis() {
    drawCube(ROBOT_CHASSIS_WIDTH, ROBOT_CHASSIS_HEIGHT, ROBOT_CHASSIS_DEPTH);
}


void drawRobotWheel() {
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
    drawFloatingDisc(ROBOT_WHEEL_RADIUS, ROBOT_WHEEL_WIDTH, 0);
    glPopMatrix();
}


void drawRobotCameraHead() {
    // --- Square camera body ---
    float cubeSize = ROBOT_CAMERA_RADIUS * 2.0f;
    drawCube(cubeSize, cubeSize, cubeSize);

    // --- Circular lens with glow effect ---
    glPushMatrix();
    {
        glTranslatef(0.0f, 0.0f, ROBOT_CAMERA_RADIUS + 0.01f);

        // Apply glow material based on light brightness
        if (g_robotLightOn && g_robotLightBrightness > 0.0f) {
            float startR = 0.5f, startG = 0.5f, startB = 0.45f;  
            float endR = 1.0f, endG = 0.9f, endB = 0.7f;         

            // Linear interpolation based on brightness
            float r = startR + g_robotLightBrightness * (endR - startR);
            float g = startG + g_robotLightBrightness * (endG - startG);
            float b = startB + g_robotLightBrightness * (endB - startB);

            GLfloat emissionColor[] = { r, g, b, 1.0f };
            setGlowingMaterial(emissionColor);
        }
        else {
            // Dark lens when light is off
            glColor3f(0.1f, 0.1f, 0.1f);
        }

        GLUquadric* quad = gluNewQuadric();
        gluDisk(quad, 0, ROBOT_CAMERA_RADIUS * 0.6f, 20, 1);
        gluDeleteQuadric(quad);
    }
    glPopMatrix();

    resetMaterial();
}

/**
 * @brief Renders the complete robot
 */
void drawRobot() {
    glPushMatrix();
    {
        glRotatef(g_robot.angleY, 0.0f, 1.0f, 0.0f);

        // Draw chassis body
        setSkimmerBodyMaterial();
        drawRobotChassis();
        // --- Draw four wheels ---
        // Front right wheel
        glPushMatrix();
        glTranslatef(ROBOT_CHASSIS_WIDTH / 2.0f + ROBOT_WHEEL_WIDTH / 2.0f, 0.0f, ROBOT_CHASSIS_DEPTH / 2.0f - ROBOT_WHEEL_RADIUS);
        glRotatef(g_robot.wheelRotation, 1.0f, 0.0f, 0.0f);
        drawRobotWheel();
        glPopMatrix();

        // Front left wheel
        glPushMatrix();
        glTranslatef(-(ROBOT_CHASSIS_WIDTH / 2.0f + ROBOT_WHEEL_WIDTH / 2.0f), 0.0f, ROBOT_CHASSIS_DEPTH / 2.0f - ROBOT_WHEEL_RADIUS);
        glRotatef(g_robot.wheelRotation, 1.0f, 0.0f, 0.0f);
        drawRobotWheel();
        glPopMatrix();

        // Rear right wheel
        glPushMatrix();
        glTranslatef(ROBOT_CHASSIS_WIDTH / 2.0f + ROBOT_WHEEL_WIDTH / 2.0f, 0.0f, -(ROBOT_CHASSIS_DEPTH / 2.0f - ROBOT_WHEEL_RADIUS));
        glRotatef(g_robot.wheelRotation, 1.0f, 0.0f, 0.0f);
        drawRobotWheel();
        glPopMatrix();

        // Rear left wheel
        glPushMatrix();
        glTranslatef(-(ROBOT_CHASSIS_WIDTH / 2.0f + ROBOT_WHEEL_WIDTH / 2.0f), 0.0f, -(ROBOT_CHASSIS_DEPTH / 2.0f - ROBOT_WHEEL_RADIUS));
        glRotatef(g_robot.wheelRotation, 1.0f, 0.0f, 0.0f);
        drawRobotWheel();
        glPopMatrix();
        // --- Draw camera head ---
        glPushMatrix();
        glTranslatef(0.0f, ROBOT_CHASSIS_HEIGHT / 2.0f + ROBOT_CAMERA_Y_OFFSET, 0.0f);
        glRotatef(0.0f, 0.0f, 1.0f, 0.0f);
        drawRobotCameraHead();
        glPopMatrix();
        // --- Draw glowing decorative stripes ---
        float r = g_glowColors[g_currentGlowColorIndex][0];
        float g = g_glowColors[g_currentGlowColorIndex][1];
        float b = g_glowColors[g_currentGlowColorIndex][2];
        GLfloat stripeEmission[] = { r, g, b, 1.0f };
        setGlowingMaterial(stripeEmission);
        const float stripe_thickness = 0.05f;
        const float stripe_offset = 0.01f;
        // Front and back stripes
        glPushMatrix();
        glTranslatef(0.0f, 0.0f, ROBOT_CHASSIS_DEPTH / 2.0f + stripe_offset);
        drawCube(ROBOT_CHASSIS_WIDTH, ROBOT_CHASSIS_HEIGHT * 0.5f, stripe_thickness);
        glPopMatrix();

        glPushMatrix();
        glTranslatef(0.0f, 0.0f, -(ROBOT_CHASSIS_DEPTH / 2.0f + stripe_offset));
        drawCube(ROBOT_CHASSIS_WIDTH, ROBOT_CHASSIS_HEIGHT * 0.5f, stripe_thickness);
        glPopMatrix();
        // Left and right stripes
        glPushMatrix();
        glTranslatef(ROBOT_CHASSIS_WIDTH / 2.0f + stripe_offset, 0.0f, 0.0f);
        drawCube(stripe_thickness, ROBOT_CHASSIS_HEIGHT * 0.5f, ROBOT_CHASSIS_DEPTH);
        glPopMatrix();

        glPushMatrix();
        glTranslatef(-(ROBOT_CHASSIS_WIDTH / 2.0f + stripe_offset), 0.0f, 0.0f);
        drawCube(stripe_thickness, ROBOT_CHASSIS_HEIGHT * 0.5f, ROBOT_CHASSIS_DEPTH);
        glPopMatrix();
    }
    glPopMatrix();
    resetMaterial();
}

void initRobot() {
    float groundLevel = CENTRAL_DISC_Y_POS + (CENTRAL_DISC_HEIGHT / 2.0f);
    g_robot.posY = groundLevel + ROBOT_WHEEL_RADIUS;
    g_robot.posX = 1.0f;
    g_robot.posZ = 1.0f;
    g_robot.angleY = 0.0f;
    g_robot.wheelRotation = 0.0f;
}
// ==========================================================
// PARTICLE SYSTEM FUNCTIONS
// ==========================================================

void initParticles() {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        waterParticles[i].active = false;
    }
}

void updateParticles(float dt) {
    static std::default_random_engine generator(static_cast<unsigned int>(time(nullptr)));
    static std::uniform_real_distribution<float> distribution(-0.1f, 0.1f);

    // Emit new particles when watering
    if (isWatering) {
        GLdouble nozzlePos[3];
        calculateNozzleWorldPosition(armBaseAngle, armLowerAngle, armUpperAngle, nozzlePos);

        int emitCount = 4;
        int emitted = 0;
        for (int i = 0; i < MAX_PARTICLES && emitted < emitCount; i++) {
            if (!waterParticles[i].active) {
                waterParticles[i].active = true;
                waterParticles[i].life = 1.5f;
                waterParticles[i].x = static_cast<float>(nozzlePos[0]);
                waterParticles[i].y = static_cast<float>(nozzlePos[1]);
                waterParticles[i].z = static_cast<float>(nozzlePos[2]);

                waterParticles[i].vx = distribution(generator);
                waterParticles[i].vy = 1.0f;
                waterParticles[i].vz = distribution(generator);

                emitted++;
            }
        }
    }

    // Update existing particles
    float groundLevel = CENTRAL_DISC_Y_POS + CENTRAL_DISC_HEIGHT / 2.0f;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (waterParticles[i].active) {
            waterParticles[i].x += waterParticles[i].vx * dt;
            waterParticles[i].y += waterParticles[i].vy * dt;
            waterParticles[i].z += waterParticles[i].vz * dt;
            waterParticles[i].vy -= GRAVITY * dt;
            waterParticles[i].life -= dt;

            if (waterParticles[i].life <= 0.0f || waterParticles[i].y < groundLevel) {
                waterParticles[i].active = false;
            }
        }
    }
}

void drawWaterParticles() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);

    glColor4f(0.6f, 0.8f, 1.0f, 0.7f);
    glPointSize(3.0f);

    glBegin(GL_POINTS);
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (waterParticles[i].active) {
            glVertex3f(waterParticles[i].x, waterParticles[i].y, waterParticles[i].z);
        }
    }
    glEnd();

    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
}

// ==========================================================
// CAMERA FUNCTIONS
// ==========================================================

/**
 * @brief Calculate the global camera state
 */
CameraState calculateGlobalCameraState() {
    CameraState state;
    float baseDistance = 20.0f;
    float baseHeight = 8.0f;

    float distance = baseDistance * g_zoomFactor;
    float height = baseHeight * g_zoomFactor;

    float angleY = g_cameraAngleY * M_PI / 180.0f;
    float angleX = g_cameraAngleX * M_PI / 180.0f;


    float cosX = cos(angleX);
    float sinX = sin(angleX);
    float cosY = cos(angleY);
    float sinY = sin(angleY);

    state.eye.x = distance * sinY * cosX;
    state.eye.y = height + distance * sinX;
    state.eye.z = distance * cosY * cosX;

    state.lookAt = vec3_create(0.0f, 0.0f, 0.0f);
    state.up = vec3_create(0.0f, 1.0f, 0.0f);

    return state;
}

/**
 * @brief Calculate the robot first-person view camera state
 */
CameraState calculateRobotCameraState() {
    CameraState state;
  
    float robotGroundY = (CENTRAL_DISC_HEIGHT / 2.0f) + ROBOT_WHEEL_RADIUS;
    float angleRad = g_robot.angleY * M_PI / 180.0f;
    float localOffsetY = (ROBOT_CHASSIS_HEIGHT / 2.0f) + ROBOT_CAMERA_Y_OFFSET;
    float localOffsetZ = ROBOT_CAMERA_RADIUS + 0.1f;
  
    float worldOffsetX = sin(angleRad) * localOffsetZ;
    float worldOffsetZ = cos(angleRad) * localOffsetZ;
  
    state.eye.x = g_robot.posX + worldOffsetX;
    state.eye.y = robotGroundY + localOffsetY;
    state.eye.z = g_robot.posZ + worldOffsetZ;
  
    state.lookAt.x = state.eye.x + sin(angleRad) * 5.0f;
    state.lookAt.y = state.eye.y;
    state.lookAt.z = state.eye.z + cos(angleRad) * 5.0f;
  
    state.up = vec3_create(0.0f, 1.0f, 0.0f);
  
    return state;
}

/**
 * @brief Smoothly interpolate between two camera states
 */
CameraState interpolateCameraState(CameraState start, CameraState end, float t) {
    t = t * t * (3.0f - 2.0f * t);
  
    CameraState result;
    result.eye = vec3_lerp(start.eye, end.eye, t);
    result.lookAt = vec3_lerp(start.lookAt, end.lookAt, t);
    result.up = vec3_lerp(start.up, end.up, t);
  
    return result;
}

void setupCamera() {
    if (g_cameraTransitioning) {
        // Use interpolated camera state
        gluLookAt(
            g_currentCameraState.eye.x, g_currentCameraState.eye.y, g_currentCameraState.eye.z,
            g_currentCameraState.lookAt.x, g_currentCameraState.lookAt.y, g_currentCameraState.lookAt.z,
            g_currentCameraState.up.x, g_currentCameraState.up.y, g_currentCameraState.up.z
        );
    }
    else if (g_isRobotView) {
        CameraState robotState = calculateRobotCameraState();
        gluLookAt(
            robotState.eye.x, robotState.eye.y, robotState.eye.z,
            robotState.lookAt.x, robotState.lookAt.y, robotState.lookAt.z,
            robotState.up.x, robotState.up.y, robotState.up.z
        );
    }
    else {
        CameraState globalState = calculateGlobalCameraState();
        gluLookAt(
            globalState.eye.x, globalState.eye.y, globalState.eye.z,
            globalState.lookAt.x, globalState.lookAt.y, globalState.lookAt.z,
            globalState.up.x, globalState.up.y, globalState.up.z
        );
    }
}


// ==========================================================
// INSTRUCTION PANEL FUNCTIONS
// ==========================================================

/**
 * @brief Draw instruction panel as 2D overlay in screen center
 */
void drawInstructionPanel() {
    // Save current matrices and attributes
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, g_windowWidth, 0, g_windowHeight);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Disable depth test and lighting
    glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Calculate panel size
    float panelHeight = g_windowHeight * 0.8f;
    float panelWidth = panelHeight * (4.0f / 3.0f);

    // Center the panel
    float panelX = (g_windowWidth - panelWidth) / 2.0f;
    float panelY = (g_windowHeight - panelHeight) / 2.0f;


    glColor4f(0.0f, 0.0f, 0.0f, 0.7f);
    glBegin(GL_QUADS);
    glVertex2f(panelX - 10, panelY - 10);
    glVertex2f(panelX + panelWidth + 10, panelY - 10);
    glVertex2f(panelX + panelWidth + 10, panelY + panelHeight + 10);
    glVertex2f(panelX - 10, panelY + panelHeight + 10);
    glEnd();

    // Draw textured panel
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, g_texControls);

    if (g_texControls != 0) {
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    }
    else {
        glDisable(GL_TEXTURE_2D);
        glColor4f(0.9f, 0.9f, 0.9f, 0.9f);
    }

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex2f(panelX, panelY);
    glTexCoord2f(1.0f, 0.0f); glVertex2f(panelX + panelWidth, panelY);
    glTexCoord2f(1.0f, 1.0f); glVertex2f(panelX + panelWidth, panelY + panelHeight);
    glTexCoord2f(0.0f, 1.0f); glVertex2f(panelX, panelY + panelHeight);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    // Restore attributes and matrices
    glPopAttrib();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
}

// ==========================================================
// SCENE RENDERING
// ==========================================================


void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    setupCamera();

    setupLights();

    drawSkyDome();
    // Central disc and its contents
    glPushMatrix();
    {
        glTranslatef(0.0f, CENTRAL_DISC_Y_POS, 0.0f);

        // Central disc platform
        glPushMatrix();
        glColor3f(0.3f, 0.6f, 0.2f);
        drawFloatingDisc(CENTRAL_DISC_RADIUS, CENTRAL_DISC_HEIGHT, g_texGroundCenter);
        glPopMatrix();

        // Robot
        glPushMatrix();
        {
            float robotGroundY = (CENTRAL_DISC_HEIGHT / 2.0f) + ROBOT_WHEEL_RADIUS;
            glTranslatef(g_robot.posX, robotGroundY, g_robot.posZ);
            drawRobot();
        }
        glPopMatrix();

        // Garden scene
        glPushMatrix();
        {
            glTranslatef(0.0f, CENTRAL_DISC_HEIGHT / 2.0f, 0.0f);
            drawGardenScene();
            drawStonePath(6.0f, -3.0f, 8.0f, 0.0f, 360.0f, 30.0f);
        }
        glPopMatrix();

        // Robotic arm
        glPushMatrix();
        {
            glTranslatef(0.0f, CENTRAL_DISC_HEIGHT / 2.0f, 0.0f);
            drawWateringArm();
        }
        glPopMatrix();
    }
    glPopMatrix();

    // Surrounding discs with buildings
    for (int i = 0; i < 5; ++i) {
        glPushMatrix();
        {
            float angle = i * 72.0f;
            glRotatef(angle, 0.0f, 1.0f, 0.0f);
            glTranslatef(0.0f, surround_heights[i], -SURROUND_DISC_DISTANCE);

            // Floating disc
            glColor3f(0.5f, 0.5f, 0.5f);
            drawFloatingDisc(SURROUND_DISC_RADIUS, SURROUND_DISC_HEIGHT, g_texGroundSurround);

            // Building on disc
            glPushMatrix();
            {
                glTranslatef(0.0f, SURROUND_DISC_HEIGHT / 2.0f, 0.0f);
                drawBuilding(i);
            }
            glPopMatrix();
        }
        glPopMatrix();
    }

    // Water particles
    drawWaterParticles();

    // Flight paths
    if (g_showFlightPath) {
        drawFlightPath(g_skimmerPath1, vec3_create(0.5f, 0.6f, 0.9f));
        drawFlightPath(g_skimmerPath2, vec3_create(0.95f, 0.4f, 0.1f));
    }

    // Animated skimmers
    drawAnimatedSkimmer(g_skimmer1_progress, g_skimmerPath1);
    drawAnimatedSkimmer(g_skimmer2_progress, g_skimmerPath2);

    // Draw instruction panel overlay if enabled
    if (g_showInstructionPanel) {
        drawInstructionPanel();
    }

    glutSwapBuffers();
}


// ==========================================================
// CALLBACK FUNCTIONS
// ==========================================================

void reshape(int width, int height) {
    g_windowWidth = width;
    g_windowHeight = height;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)width / (float)height, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        g_mouseLeftDown = (state == GLUT_DOWN);
        if (g_mouseLeftDown) {
            g_mouseX = x;
            g_mouseY = y;
        }
    }
    else if (button == 3) {  // Scroll up - zoom in
        g_zoomFactor *= 0.9f;
        if (g_zoomFactor < 0.3f) g_zoomFactor = 0.3f;
        glutPostRedisplay();
    }
    else if (button == 4) {  // Scroll down - zoom out
        g_zoomFactor *= 1.1f;
        if (g_zoomFactor > 1.8f) g_zoomFactor = 1.8f;
        glutPostRedisplay();
    }
}

void motion(int x, int y) {
    if (g_mouseLeftDown && !g_isRobotView) {
        int dx = x - g_mouseX;
        int dy = y - g_mouseY;
        g_cameraAngleY += dx * 0.5f;
        g_cameraAngleX += dy * 0.5f;

        // Clamp vertical rotation
        if (g_cameraAngleX > 90.0f) g_cameraAngleX = 90.0f;
        if (g_cameraAngleX < -90.0f) g_cameraAngleX = -90.0f;

        g_mouseX = x;
        g_mouseY = y;
        glutPostRedisplay();
    }
}

void keyboard(unsigned char key, int x, int y) {
    float angleRad = g_robot.angleY * M_PI / 180.0f;
    float nextX = g_robot.posX;
    float nextZ = g_robot.posZ;
    float oldLowerAngle = armLowerAngle;
    float oldUpperAngle = armUpperAngle;
    float oldBaseAngle = armBaseAngle;

    // Robot movement
    if (key == 'w' || key == 'W') {
        nextX += sin(angleRad) * ROBOT_MOVE_SPEED;
        nextZ += cos(angleRad) * ROBOT_MOVE_SPEED;
        g_robot.wheelRotation -= 15.0f;
    }
    else if (key == 's' || key == 'S') {
        nextX -= sin(angleRad) * ROBOT_MOVE_SPEED;
        nextZ -= cos(angleRad) * ROBOT_MOVE_SPEED;
        g_robot.wheelRotation += 15.0f;
    }
    else if (key == 'a' || key == 'A') {
        g_robot.angleY += ROBOT_ROTATE_SPEED;
    }
    else if (key == 'd' || key == 'D') {
        g_robot.angleY -= ROBOT_ROTATE_SPEED;
    }
    else if (key == 'c' || key == 'C') {
        if (g_cameraTransitioning) {
            return;
        }

        g_cameraTransitioning = true;
        g_cameraTransitionProgress = 0.0f;

        if (g_isRobotView) {
            g_cameraStartState = calculateRobotCameraState();
            g_cameraEndState = calculateGlobalCameraState();
        }
        else {
            g_cameraStartState = calculateGlobalCameraState();
            g_cameraEndState = calculateRobotCameraState();
        }

        g_currentCameraState = g_cameraStartState;
    }
    else if (key == 'p' || key == 'P') {
        isWatering = true;
    }
    else if (key == 't' || key == 'T') {
        g_showFlightPath = !g_showFlightPath;
    }
    else if (key == 'k' || key == 'K') {
        g_currentGlowColorIndex = (g_currentGlowColorIndex + 1) % TOTAL_COLORS;
    }
    // Instruction panel
    else if (key == 'i' || key == 'I') {
        g_showInstructionPanel = !g_showInstructionPanel;
    }
    // Robot light control
    else if (key == 'n' || key == 'N') {
        g_envLightOn = !g_envLightOn;
        g_currentSkyIndex = (g_currentSkyIndex + 1) % 2;
    }
    else if (key == '+' || key == '=') {
        g_robotLightBrightness += 0.1f;
        if (g_robotLightBrightness > 4.0f) g_robotLightBrightness = 4.0f;
    }
    else if (key == '-' || key == '_') {
        g_robotLightBrightness -= 0.1f;
        if (g_robotLightBrightness < 0.0f) g_robotLightBrightness = 0.0f;
    }
    // Arm control
    else if (key == '1') {
        armLowerAngle += 5.0f;
    }
    else if (key == '2') {
        armLowerAngle -= 5.0f;
    }
    else if (key == '3') {
        armUpperAngle += 5.0f;
    }
    else if (key == '4') {
        armUpperAngle -= 5.0f;
    }
    else if (key == '5') {
        armBaseAngle += 5.0f;
    }
    else if (key == '6') {
        armBaseAngle -= 5.0f;
    }

    // Validate arm position
    GLdouble nozzlePos[3];
    calculateNozzleWorldPosition(armBaseAngle, armLowerAngle, armUpperAngle, nozzlePos);
    float groundLevel = CENTRAL_DISC_Y_POS + CENTRAL_DISC_HEIGHT / 2.0f;
    if (nozzlePos[1] < groundLevel + 0.1f) {
        armLowerAngle = oldLowerAngle;
        armUpperAngle = oldUpperAngle;
        armBaseAngle = oldBaseAngle;
    }

    // Validate robot position
    if (checkRobotBoundary(nextX, nextZ)) {
        g_robot.posX = nextX;
        g_robot.posZ = nextZ;
    }

    glutPostRedisplay();
}

void keyboardUp(unsigned char key, int x, int y) {
    if (key == 'p' || key == 'P') {
        isWatering = false;
    }
}

void idle() {
    static int lastTime = glutGet(GLUT_ELAPSED_TIME);
    int currentTime = glutGet(GLUT_ELAPSED_TIME);
    float dt = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;

    updateParticles(dt);
    if (g_cameraTransitioning) {
        g_cameraTransitionProgress += dt / CAMERA_TRANSITION_DURATION;

        if (g_cameraTransitionProgress >= 1.0f) {
            g_cameraTransitioning = false;
            g_cameraTransitionProgress = 1.0f;
            g_isRobotView = !g_isRobotView;
        }

        g_currentCameraState = interpolateCameraState(
            g_cameraStartState,
            g_cameraEndState,
            g_cameraTransitionProgress
        );
    }
    // Update skimmer positions with looping
    g_skimmer1_progress += SKIMMER1_SPEED * dt;
    if (g_skimmer1_progress >= g_skimmerPath1.size()) {
        g_skimmer1_progress -= g_skimmerPath1.size();
    }

    g_skimmer2_progress += SKIMMER2_SPEED * dt;
    if (g_skimmer2_progress >= g_skimmerPath2.size()) {
        g_skimmer2_progress -= g_skimmerPath2.size();
    }

    glutPostRedisplay();
}

// ==========================================================
// INITIALIZATION AND MAIN
// ==========================================================

void onMenu(int item) {
    switch (item) {
    case MENU_TOGGLE_DAY_NIGHT:
        g_envLightOn = !g_envLightOn;
        g_currentSkyIndex = (g_currentSkyIndex + 1) % 2;
        break;

    case MENU_TOGGLE_PATH:
        g_showFlightPath = !g_showFlightPath;
        break;

    case MENU_TOGGLE_INSTRUCTIONS:
        g_showInstructionPanel = !g_showInstructionPanel;
        break;

    case MENU_COLOR_CYAN:   g_currentGlowColorIndex = 0; break;
    case MENU_COLOR_ORANGE:    g_currentGlowColorIndex = 1; break;
    case MENU_COLOR_PURPLE:  g_currentGlowColorIndex = 2; break;
    case MENU_COLOR_GREEN:   g_currentGlowColorIndex = 3; break;
    case MENU_COLOR_YELLO: g_currentGlowColorIndex = 4; break;
    }

    glutPostRedisplay(); 
}

void setupMenus() {
    int subMenuColor = glutCreateMenu(onMenu);
    glutAddMenuEntry("Blue", MENU_COLOR_CYAN);
    glutAddMenuEntry("Orange", MENU_COLOR_ORANGE);
    glutAddMenuEntry("Purple", MENU_COLOR_PURPLE);
    glutAddMenuEntry("Green", MENU_COLOR_GREEN);
    glutAddMenuEntry("Yellow", MENU_COLOR_YELLO);

    int mainMenu = glutCreateMenu(onMenu);
    glutAddMenuEntry("Toggle Day/Night (N)", MENU_TOGGLE_DAY_NIGHT);
    glutAddMenuEntry("Toggle Flight Path (T)", MENU_TOGGLE_PATH);
    glutAddMenuEntry("Toggle Instructions (I)", MENU_TOGGLE_INSTRUCTIONS);
    glutAddSubMenu("Glow Color (K)", subMenuColor);

    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void initGL() {
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    /*glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);*/
    initRobot();

    initParticles();
}

void initTextures() {
    g_texTreeBark = loadTexture("texture/bark.bmp");
    g_texTreeLeaf = loadTexture("texture/leaf.bmp");
    g_texGroundCenter = loadTexture("texture/grass.bmp");
    g_texGroundSurround = loadTexture("texture/stone.bmp");
    g_texBush = loadTexture("texture/bush.bmp");
    g_texSkyDay = loadTexture("texture/day.bmp");
    g_texSkyNight = loadTexture("texture/night.bmp");
    g_texControls = loadTexture("texture/controls.bmp");
}

int main(int argc, char** argv) {

    std::cout << "Robot Movement:\n";
    std::cout << "  W / w : Move forward\n";
    std::cout << "  S / s : Move backward\n";
    std::cout << "  A / a : Turn left\n";
    std::cout << "  D / d : Turn right\n";
    std::cout << "Camera:\n";
    std::cout << "  C / c : Toggle robot first-person view\n";
    std::cout << "Robotic Arm:\n";
    std::cout << "  1 / 2: Raise/Lower lower arm\n";
    std::cout << "  3 / 4 : Raise/Lower upper arm\n";
    std::cout << "  5 / 6: Rotate arm base\n";
    std::cout << "Watering:\n";
    std::cout << "  P / p : Start watering (hold)\n";
    std::cout << "Lighting:\n";
    std::cout << "  N / n : Toggle day/night mode\n";
    std::cout << "  + / = : Increase robot headlight brightness\n";
    std::cout << "  - / _ : Decrease robot headlight brightness\n";
    std::cout << "Visuals:\n";
    std::cout << "  T / t : Toggle flight path display\n";
    std::cout << "  K / k : Switch glow color\n";
    std::cout << "  I / i : Toggle instruction panel\n";
    std::cout << "Mouse:\n";
    std::cout << "  Left button drag : Rotate global camera\n";
    std::cout << "  Scroll wheel : Zoom in/out\n";
    std::cout << "Menu (Right Mouse Button):\n";
    std::cout << "  Change glow color, toggle day/night, toggle flight path, show instructions\n";

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Future City");

    initGL();
    initPaths();
    generateFractalTreeGrammar();
    initTextures();
    setupMenus();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}