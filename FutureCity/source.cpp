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
const float ROBOT_CHASSIS_WIDTH = 0.8f;    // 机器人底盘宽度 (X轴)
const float ROBOT_CHASSIS_HEIGHT = 0.4f;   // 机器人底盘高度 (Y轴)
const float ROBOT_CHASSIS_DEPTH = 1.2f;    // 机器人底盘深度 (Z轴)
const float ROBOT_WHEEL_RADIUS = 0.3f;     // 机器人车轮半径
const float ROBOT_WHEEL_WIDTH = 0.15f;     // 机器人车轮宽度
const float ROBOT_CAMERA_RADIUS = 0.25f;    // 摄像头头部半径
const float ROBOT_CAMERA_Y_OFFSET = 0.3f;  // 摄像头距离底盘的高度

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
const int FRACTAL_TREE_ITERATIONS = 3;        // L-System 迭代次数 (建议 3-5)
const float TREE_INITIAL_HEIGHT = 1.5f;       // 初始树干高度
const float TREE_INITIAL_RADIUS = 0.1f;       // 初始树干半径
const float TREE_HEIGHT_DECAY = 0.7f;         // 每次迭代，高度衰减系数
const float TREE_RADIUS_DECAY = 0.65f;        // 每次迭代，半径衰减系数
const float TREE_LEAF_SIZE = 0.4f;            // 叶片大小
const float TREE_BRANCH_ANGLE = 25.0f;        // 树枝分叉角度
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

// Robot state
Robot g_robot;
//float g_robotCameraAngleY = 0.0f;
bool g_robotLightOn = true;         // 车灯开关状态
float g_robotLightBrightness = 0.8f;   // 车灯亮度 (0.0 到 1.0)

// Global lighting state
bool g_envLightOn = true;

// Robotic arm state
float armBaseAngle = 0.0f;
float armLowerAngle = 45.0f;
float armUpperAngle = 60.0f;
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
std::string g_fractalTreeGrammar; // 存储生成的L-System指令字符串

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



// ==========================================================
// FUNCTION DECLARATIONS
// ==========================================================
void initTextures();
// Initialization
void initGL();
void initPaths();
void initParticles();

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
void drawFlowerPetal(vec3 v_center, vec3 v_edge1, vec3 v_edge2);
void drawFlower(vec3 petalColor);
void drawGardenScene();
void drawFlowerGarden();

// Robotic arm
void drawArmBase();
void drawArmJoint();
void drawArmSegment();
void drawNozzle();
void drawWateringArm();
void calculateNozzleWorldPosition(float baseRot, float lowerArmRot, float upperArmRot, GLdouble outPos[3]);

// Building
void drawFuturisticBuilding(float baseSize, float height, int numWindowFloors);

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
// ==========================================================
// TEXTURE LOADING FUNCTIONS
// ==========================================================

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
    int width = *(int*)&(header[0x12]);
    int height = *(int*)&(header[0x16]);
    unsigned int dataPos = *(unsigned int*)&(header[0x0A]);
    int imageSize = width * height * 3; 
    if (dataPos == 0) dataPos = 54;

    int rowSizePadded = (width * 3 + 3) & (~3);
    int padding = rowSizePadded - (width * 3); 
    unsigned char* data = new unsigned char[imageSize];

    fseek(file, dataPos, SEEK_SET);

    for (int i = 0; i < height; i++) {
        fread(data + (i * width * 3), 1, width * 3, file);
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

    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_BGR_EXT, GL_UNSIGNED_BYTE, data);

    delete[] data;

    std::cout << "[Texture Success] Loaded: " << filename << " (ID: " << textureID << ")" << std::endl;
    return textureID;
}

// ==========================================================
// ROBOT LIGHTING CALCULATION FUNCTIONS
// ==========================================================

/**
 * @brief Calculate the world position of the robot's headlight.
 * The light emanates from the camera head position.
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

    // Move slightly forward from camera center
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
 * The light points in the direction the robot is facing.
 */
void calculateRobotLightWorldDirection(GLdouble outDir[3]) {
    float angleRad = g_robot.angleY * M_PI / 180.0f;

    // Direction vector in world space (robot's forward direction)
    outDir[0] = sin(angleRad);
    outDir[1] = 0.0f;  // Horizontal spotlight
    outDir[2] = cos(angleRad);
}

/**
 * @brief Setup all lights in the scene (global light + robot spotlight).
 */
void setupLights() {
    // --- Setup LIGHT0 (Main global ambient light) ---
    if (g_envLightOn) {
        glEnable(GL_LIGHT0);
        GLfloat light0_ambient[] = { 0.15f, 0.15f, 0.15f, 1.0f };
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

        // Calculate robot's world position
        float robotGroundY = (CENTRAL_DISC_HEIGHT / 2.0f) + ROBOT_WHEEL_RADIUS;
        float angleRad = g_robot.angleY * M_PI / 180.0f;

        // Light position: at the camera head, in world coordinates
        GLfloat lightPosX = g_robot.posX + sin(angleRad) * ROBOT_CAMERA_RADIUS;
        GLfloat lightPosY = robotGroundY + (ROBOT_CHASSIS_HEIGHT / 2.0f) + ROBOT_CAMERA_Y_OFFSET;
        GLfloat lightPosZ = g_robot.posZ + cos(angleRad) * ROBOT_CAMERA_RADIUS;

        GLfloat light1_position[] = { lightPosX, lightPosY, lightPosZ, 1.0f };

        // Light direction: pointing forward in the direction the robot faces
        GLfloat light1_direction[] = {
            sin(angleRad),
            0.0f,  // Horizontal spotlight
            cos(angleRad)
        };

        // Set light properties
        glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
        glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light1_direction);
        glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 35.0f);      // Spotlight cone angle (degrees)
        glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 15.0f);    // Spotlight focus

        // Set light attenuation
        glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0.5f);
        glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.08f);
        glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.01f);

        // Calculate light color based on brightness
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

// Generic material setter
void setMaterial(const GLfloat* ambient, const GLfloat* diffuse, const GLfloat* specular, float shininess, const GLfloat* emission) {
    glDisable(GL_COLOR_MATERIAL);
    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);
}

// Building frame material (dark gray metallic)
void setBuildingFrameMaterial() {
    GLfloat ambient[] = { 0.15f, 0.15f, 0.2f, 1.0f };
    GLfloat diffuse[] = { 0.2f, 0.2f, 0.25f, 1.0f };
    GLfloat specular[] = { 0.4f, 0.4f, 0.5f, 1.0f };
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(ambient, diffuse, specular, 30.0f, emission);
}

// Glowing material (for windows and rings)
void setGlowingMaterial(const GLfloat* emissionColor) {
    GLfloat black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(black, black, black, 0.0f, emissionColor);
}

void setSkimmerBodyMaterial() {
    GLfloat ambient[] = { 0.1f, 0.1f, 0.15f, 1.0f };
    GLfloat diffuse[] = { 0.85f, 0.85f, 0.95f, 1.0f };
    GLfloat specular[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(ambient, diffuse, specular, 3.0f, emission);
}

void setTreeTrunkMaterial() {
    GLfloat ambient[] = { 0.4f, 0.25f, 0.15f, 1.0f };
    GLfloat diffuse[] = { 0.5f, 0.35f, 0.2f, 1.0f };
    GLfloat specular[] = { 0.1f, 0.05f, 0.0f, 1.0f };
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    setMaterial(ambient, diffuse, specular, 5.0f, emission);
}

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

    // --- 开启自动纹理坐标生成 ---
    gluQuadricTexture(quadric, GL_TRUE);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // 如果纹理加载失败，使用白色混合
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
 * @brief 生成 L-System 分形树的指令字符串。
 * F: 向前画树干和叶子
 * [: 保存当前状态 (位置和朝向)
 * ]: 恢复上一个状态
 * +: 绕X轴正向旋转 (向上抬头)
 * -: 绕X轴负向旋转 (向下低头)
 * &: 绕Y轴正向旋转 (向左偏航)
 * ^: 绕Y轴负向旋转 (向右偏航)
 * /: 绕Z轴正向旋转 (向左翻滚)
 * \: 绕Z轴负向旋转 (向右翻滚)
 */
void generateFractalTreeGrammar() {
    std::string axiom = "F"; // 初始公理：一根树干
    // 重写规则：将每个 'F' 替换为更复杂的结构
    std::string rule = "F[+F&F][-F^F][/F\F]";

    std::string currentString = axiom;

    for (int i = 0; i < FRACTAL_TREE_ITERATIONS; ++i) {
        std::string nextString = "";
        for (char c : currentString) {
            if (c == 'F') {
                nextString += rule; // 应用规则
            }
            else {
                nextString += c; // 保留其他字符 ([, ], +, -, etc.)
            }
        }
        currentString = nextString;
    }
    g_fractalTreeGrammar = currentString;
    // Optional: Print the grammar to see the complexity
    // std::cout << "Generated Grammar (" << FRACTAL_TREE_ITERATIONS << " iterations):\n" << g_fractalTreeGrammar << std::endl;
}

void drawTreeLeaf(float size) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, g_texTreeLeaf);

    if (g_texTreeLeaf != 0) glColor3f(1.0f, 1.0f, 1.0f);
    else glDisable(GL_TEXTURE_2D);

    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);

    // 映射纹理坐标 (0,0) 到 (1,1)
    glTexCoord2f(0.5f, 0.0f); glVertex3f(0, 0, 0);             // 底部中心
    glTexCoord2f(1.0f, 0.5f); glVertex3f(size / 2.0f, size, 0); // 右侧
    glTexCoord2f(0.5f, 1.0f); glVertex3f(0, size * 2.0f, 0);    // 顶部
    glTexCoord2f(0.0f, 0.5f); glVertex3f(-size / 2.0f, size, 0);// 左侧
    glEnd();

    glDisable(GL_TEXTURE_2D);
}

// 绘制一段树干 (一个圆柱体)
void drawTreeBranch(float radius, float height) {
    glPushMatrix();
    glRotatef(-90, 1.0f, 0.0f, 0.0f);

    GLUquadric* quad = gluNewQuadric();
    gluQuadricTexture(quad, GL_TRUE); // 开启纹理

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, g_texTreeBark);

    if (g_texTreeBark != 0) glColor3f(1.0f, 1.0f, 1.0f); // 有贴图时用白色
    else glDisable(GL_TEXTURE_2D); // 没贴图用原来的材质颜色

    gluCylinder(quad, radius, radius * 0.8f, height, 8, 1);

    glDisable(GL_TEXTURE_2D);
    gluDeleteQuadric(quad);
    glPopMatrix();
}
/**
 * @brief 根据生成的 L-System 字符串递归绘制分形树。
 */
void drawFractalTree() {
    if (g_fractalTreeGrammar.empty()) return;
    srand(0);
    // 初始化绘制参数
    float currentHeight = TREE_INITIAL_HEIGHT;
    float currentRadius = TREE_INITIAL_RADIUS;

    // 遍历指令字符串
    for (char c : g_fractalTreeGrammar) {
        switch (c) {
        case 'F': // 向前画树干，并在末端画一片叶子
            setTreeTrunkMaterial();
            drawTreeBranch(currentRadius, currentHeight);
            // 将坐标系移动到树枝顶端
            glTranslatef(0.0f, currentHeight, 0.0f);

            // 在新树枝的末端画一片叶子
            setTreeLeafMaterial();
            // 随机旋转一下叶子，让它看起来更自然
            glPushMatrix();
            glRotatef(rand() % 360, 0.0f, 1.0f, 0.0f);
            glRotatef((rand() % 40) - 20, 1.0f, 0.0f, 0.0f);
            drawTreeLeaf(TREE_LEAF_SIZE);
            glPopMatrix();
            break;

        case '[': // 保存当前状态
            glPushMatrix();
            // 每次分叉，后续的树枝会变短变细
            currentHeight *= TREE_HEIGHT_DECAY;
            currentRadius *= TREE_RADIUS_DECAY;
            break;

        case ']': // 恢复上一个状态
            glPopMatrix();
            // 恢复之前的尺寸
            currentHeight /= TREE_HEIGHT_DECAY;
            currentRadius /= TREE_RADIUS_DECAY;
            break;

            // --- 旋转指令 ---
        case '+': glRotatef(TREE_BRANCH_ANGLE, 1.0f, 0.0f, 0.0f); break;
        case '-': glRotatef(-TREE_BRANCH_ANGLE, 1.0f, 0.0f, 0.0f); break;
        case '&': glRotatef(TREE_BRANCH_ANGLE, 0.0f, 1.0f, 0.0f); break;
        case '^': glRotatef(-TREE_BRANCH_ANGLE, 0.0f, 1.0f, 0.0f); break;
        case '/': glRotatef(TREE_BRANCH_ANGLE, 0.0f, 0.0f, 1.0f); break;
        case '\\': glRotatef(-TREE_BRANCH_ANGLE, 0.0f, 0.0f, 1.0f); break;
        default:
            break;
        }
    }
    resetMaterial(); 
}
// Draw an icosahedron-based bush
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
    glBindTexture(GL_TEXTURE_2D, g_texBush);

    if (g_texBush != 0) glColor3f(1.0f, 1.0f, 1.0f);
    else glDisable(GL_TEXTURE_2D);

    glBegin(GL_TRIANGLES);
    for (int i = 0; i < 20; i++) {
        const GLfloat* ptr1 = vertices[faces[i][0]];
        const GLfloat* ptr2 = vertices[faces[i][1]];
        const GLfloat* ptr3 = vertices[faces[i][2]];

        // --- 独立面贴图逻辑 ---

        glNormal3fv(ptr1);
        if (i % 2 == 0) glTexCoord2f(0.5f, 1.0f); // 顶
        else            glTexCoord2f(0.0f, 0.0f); // 左下
        glVertex3fv(ptr1);

        glNormal3fv(ptr2);
        if (i % 2 == 0) glTexCoord2f(0.0f, 0.0f); // 左下
        else            glTexCoord2f(1.0f, 0.0f); // 右下
        glVertex3fv(ptr2);

        glNormal3fv(ptr3);
        if (i % 2 == 0) glTexCoord2f(1.0f, 0.0f); // 右下
        else            glTexCoord2f(0.5f, 1.0f); // 顶
        glVertex3fv(ptr3);
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);
}


void drawFlowerPetal(vec3 v_center, vec3 v_edge1, vec3 v_edge2) {
    const float PETAL_HEIGHT = 0.25f;

    glBegin(GL_TRIANGLES);
    glNormal3f(v_center.x, v_center.y, v_center.z); glVertex3f(v_center.x, v_center.y, v_center.z);
    glNormal3f(v_edge1.x, v_edge1.y, v_edge1.z); glVertex3f(v_edge1.x, v_edge1.y, v_edge1.z);
    glNormal3f(v_edge2.x, v_edge2.y, v_edge2.z); glVertex3f(v_edge2.x, v_edge2.y, v_edge2.z);
    glEnd();

    vec3 mid_point = vec3_scale(vec3_add(v_edge1, v_edge2), 0.5f);
    vec3 outward_direction = vec3_normalize(mid_point);
    vec3 translation = vec3_scale(outward_direction, PETAL_HEIGHT);
    vec3 v_outer1 = vec3_add(v_edge1, translation);
    vec3 v_outer2 = vec3_add(v_edge2, translation);


    glBegin(GL_QUADS);
    glNormal3f(v_edge1.x, v_edge1.y, v_edge1.z);
    glVertex3fv(&v_edge1.x); 

    glNormal3f(v_edge2.x, v_edge2.y, v_edge2.z);
    glVertex3fv(&v_edge2.x); 

    vec3 normal_outer2 = vec3_normalize(v_outer2);
    glNormal3fv(&normal_outer2.x);
    glVertex3fv(&v_outer2.x);

    vec3 normal_outer1 = vec3_normalize(v_outer1);
    glNormal3fv(&normal_outer1.x);
    glVertex3fv(&v_outer1.x); 

    glEnd();
}

// Draw a flower with multiple petals
void drawFlower(vec3 petalColor) {
    const float t = (1.0f + sqrt(5.0f)) / 2.0f;
    const float r = 1.0f / sqrt(1.0f * 1.0f + t * t);
    const float v1 = 1.0f * r, v2 = t * r;

    static const GLfloat vertices[12][3] = {
        {-v1, v2, 0}, {v1, v2, 0}, {-v1, -v2, 0}, {v1, -v2, 0},
        {0, -v1, v2}, {0, v1, v2}, {0, -v1, -v2}, {0, v1, -v2},
        {v2, 0, -v1}, {v2, 0, v1}, {-v2, 0, -v1}, {-v2, 0, v1}
    };
    static const GLint faces[20][3] = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}
    };

    glColor3f(petalColor.x, petalColor.y, petalColor.z);
    for (int i = 0; i < 5; i++) {
        vec3 vC = vec3_create(vertices[faces[i][0]][0], vertices[faces[i][0]][1], vertices[faces[i][0]][2]);
        vec3 vA = vec3_create(vertices[faces[i][1]][0], vertices[faces[i][1]][1], vertices[faces[i][1]][2]);
        vec3 vB = vec3_create(vertices[faces[i][2]][0], vertices[faces[i][2]][1], vertices[faces[i][2]][2]);
        drawFlowerPetal(vC, vA, vB);
    }
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
        {-2.0f, 0.0f, 3.0f},
        {-4.0f, 0.0f, 2.0f},
        {-2.8f, 0.0f, 1.3f},
        {-4.5f, 0.0f, 0.5f},
        {-3.8f, 0.0f, -1.6f}
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

// Draw flower garden
void drawFlowerGarden() {
    vec3 flowerColor = { 1.0f, 0.6f, 0.8f };

    vec3 flowerPositions[] = {
        {-1.5f, 0.5f, 4.5f},
        {-0.7f, 0.5f, 4.0f},
        {-0.6f, 0.5f, 3.2f},
    };
    vec3 flowerScales[] = {
        {0.35f, 0.35f, 0.35f},
        {0.4f, 0.4f, 0.4f},
        {0.3f, 0.3f, 0.3f},
    };
    vec3 flowerRotations[] = {
        {130.0f, 70.0f, 80.0f},
        {80.0f, 20.0f, 90.0f},
        {30.0f, 30.0f, 80.0f},
    };

    for (int i = 0; i < 0; i++) {
        glPushMatrix();
        glTranslatef(flowerPositions[i].x, flowerPositions[i].y, flowerPositions[i].z);
        glRotatef(flowerRotations[i].x, 1.0f, 0.0f, 0.0f); 
        glRotatef(flowerRotations[i].y, 0.0f, 1.0f, 0.0f);
        glRotatef(flowerRotations[i].z, 0.0f, 0.0f, 1.0f); 
        glScalef(flowerScales[i].x, flowerScales[i].y, flowerScales[i].z);
        drawFlower(flowerColor);
        glPopMatrix();
    }

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

// Draw futuristic building with glowing windows
void drawFuturisticBuilding(float baseSize, float height, int numWindowFloors) {
    // Draw building frame
    setBuildingFrameMaterial();
    glPushMatrix();
    glTranslatef(0.0f, height / 2.0f, 0.0f);
    drawCube(baseSize, height, baseSize);
    glPopMatrix();

    // Draw glowing windows
    GLfloat windowEmission[] = { 0.5f, 0.8f, 1.0f, 1.0f };
    setGlowingMaterial(windowEmission);

    float windowHeight = height / (float)numWindowFloors * 0.6f;
    float floorHeight = height / (float)numWindowFloors;
    float windowDepthOffset = baseSize * 0.505f;

    for (int i = 0; i < numWindowFloors; ++i) {
        float y_pos = i * floorHeight + floorHeight * 0.2f;

        for (int side = 0; side < 4; ++side) {
            glPushMatrix();
            glRotatef(90.0f * side, 0.0f, 1.0f, 0.0f);
            glTranslatef(0.0f, y_pos, windowDepthOffset);
            drawCube(baseSize * 0.8f, windowHeight, 0.01f);
            glPopMatrix();
        }
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

// Draw glowing decorative ring on fuselage
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
    // --- 1. Fuselage ---
    setSkimmerBodyMaterial();
    glPushMatrix();
    glScalef(SKIMMER_WIDTH, SKIMMER_WIDTH, SKIMMER_LENGTH);
    glutSolidSphere(1.0, 16, 12);
    glPopMatrix();

    // --- 2. Wings ---
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

    // --- 3. Glowing Rings ---
    GLfloat ringEmission[] = { 0.5f, 0.8f, 1.0f, 1.0f };
    setGlowingMaterial(ringEmission);
    drawGlowingRing(SKIMMER_LENGTH * 0.3f, 0.2f);
    drawGlowingRing(SKIMMER_LENGTH * -0.3f, 0.2f);

    // --- 4. Reset material state ---
    resetMaterial();
}
// Catmull-Rom spline interpolation
vec3 getCatmullRomPoint(vec3 p0, vec3 p1, vec3 p2, vec3 p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;

    vec3 result;
    result.x = 0.5f * ((2.0f * p1.x) +
        (-p0.x + p2.x) * t +
        (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 +
        (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3);

    result.y = 0.5f * ((2.0f * p1.y) +
        (-p0.y + p2.y) * t +
        (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 +
        (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3);

    result.z = 0.5f * ((2.0f * p1.z) +
        (-p0.z + p2.z) * t +
        (2.0f * p0.z - 5.0f * p1.z + 4.0f * p2.z - p3.z) * t2 +
        (-p0.z + 3.0f * p1.z - 3.0f * p2.z + p3.z) * t3);

    return result;
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

    return getCatmullRomPoint(path[p0_idx], path[p1_idx], path[p2_idx], path[p3_idx], t);
}

// Initialize flight paths
void initPaths() {
    float r = SURROUND_DISC_DISTANCE;

    // Path 1
    g_skimmerPath1.push_back(vec3_create(r + 8.0f, 3.0f, 0.0f));
    g_skimmerPath1.push_back(vec3_create(0.0f, 6.0f, r + 6.0f));
    g_skimmerPath1.push_back(vec3_create(-r + 7.0f, 4.0f, r - 8.0f));
    g_skimmerPath1.push_back(vec3_create(-r - 4.0f, 5.0f, 0.0f));
    g_skimmerPath1.push_back(vec3_create(-r, 6.0f, -r + 2.0f));
    g_skimmerPath1.push_back(vec3_create(0.0f, 10.0f, -r));

    // Path 2
    g_skimmerPath2.push_back(vec3_create(r - 7.0f, 3.0f, 0.0f));
    g_skimmerPath2.push_back(vec3_create(0.0f, 6.0f, r + 3.0f));
    g_skimmerPath2.push_back(vec3_create(-r - 8.0f, 4.0f, r - 8.0f));
    g_skimmerPath2.push_back(vec3_create(-r - 4.0f, 6.0f, 0.0f));
    g_skimmerPath2.push_back(vec3_create(-r, 6.0f, -r + 1.0f));
    g_skimmerPath2.push_back(vec3_create(1.0f, 3.0f, -r - 5.0f));
}

// ==========================================================
// ROBOT FUNCTIONS
// ==========================================================

// Check if robot position is within central disc boundary
bool checkRobotBoundary(float x, float z) {
    float distanceFromCenter = sqrt(x * x + z * z);
    return distanceFromCenter < CENTRAL_DISC_RADIUS - (ROBOT_CHASSIS_WIDTH / 2.0f);
}

/**
 * @brief 绘制机器人的底盘。
 */
void drawRobotChassis() {
    drawCube(ROBOT_CHASSIS_WIDTH, ROBOT_CHASSIS_HEIGHT, ROBOT_CHASSIS_DEPTH);
}

/**
 * @brief 绘制机器人的一个车轮。
 */
void drawRobotWheel() {
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
    drawFloatingDisc(ROBOT_WHEEL_RADIUS, ROBOT_WHEEL_WIDTH,0);
    glPopMatrix();
}

/**
 * @brief 绘制机器人的摄像头头部。
 */
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
            // Interpolate between dim and bright colors
            float startR = 0.5f, startG = 0.5f, startB = 0.45f;  // Dim color
            float endR = 1.0f, endG = 0.9f, endB = 0.7f;         // Bright color

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
 * @brief 绘制一个机器人。
 */
void drawRobot() {

    glPushMatrix();
    {

        glRotatef(g_robot.angleY, 0.0f, 1.0f, 0.0f);
        setSkimmerBodyMaterial();
        drawRobotChassis();


        // 前右轮
        glPushMatrix();
        glTranslatef(ROBOT_CHASSIS_WIDTH / 2.0f + ROBOT_WHEEL_WIDTH / 2.0f, 0.0f, ROBOT_CHASSIS_DEPTH / 2.0f - ROBOT_WHEEL_RADIUS);
        glRotatef(g_robot.wheelRotation, 1.0f, 0.0f, 0.0f);
        drawRobotWheel();
        glPopMatrix();
        // 前左轮
        glPushMatrix();
        glTranslatef(-(ROBOT_CHASSIS_WIDTH / 2.0f + ROBOT_WHEEL_WIDTH / 2.0f), 0.0f, ROBOT_CHASSIS_DEPTH / 2.0f - ROBOT_WHEEL_RADIUS);
        glRotatef(g_robot.wheelRotation, 1.0f, 0.0f, 0.0f);
        drawRobotWheel();
        glPopMatrix();
        // 后右轮
        glPushMatrix();
        glTranslatef(ROBOT_CHASSIS_WIDTH / 2.0f + ROBOT_WHEEL_WIDTH / 2.0f, 0.0f, -(ROBOT_CHASSIS_DEPTH / 2.0f - ROBOT_WHEEL_RADIUS));
        glRotatef(g_robot.wheelRotation, 1.0f, 0.0f, 0.0f);
        drawRobotWheel();
        glPopMatrix();
        // 后左轮
        glPushMatrix();
        glTranslatef(-(ROBOT_CHASSIS_WIDTH / 2.0f + ROBOT_WHEEL_WIDTH / 2.0f), 0.0f, -(ROBOT_CHASSIS_DEPTH / 2.0f - ROBOT_WHEEL_RADIUS));
        glRotatef(g_robot.wheelRotation, 1.0f, 0.0f, 0.0f);
        drawRobotWheel();
        glPopMatrix();

        glPushMatrix();
        glTranslatef(0.0f, ROBOT_CHASSIS_HEIGHT / 2.0f + ROBOT_CAMERA_Y_OFFSET, 0.0f);
        glRotatef(0.0f, 0.0f, 1.0f, 0.0f);
        drawRobotCameraHead();
        glPopMatrix();


        GLfloat stripeEmission[] = { 0.5f, 0.8f, 1.0f, 1.0f };
        setGlowingMaterial(stripeEmission);

        const float stripe_thickness = 0.05f;
        const float stripe_offset = 0.01f;

        // 前后条带
        glPushMatrix();
        glTranslatef(0.0f, 0.0f, ROBOT_CHASSIS_DEPTH / 2.0f + stripe_offset);
        drawCube(ROBOT_CHASSIS_WIDTH, ROBOT_CHASSIS_HEIGHT * 0.5f, stripe_thickness);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0.0f, 0.0f, -(ROBOT_CHASSIS_DEPTH / 2.0f + stripe_offset));
        drawCube(ROBOT_CHASSIS_WIDTH, ROBOT_CHASSIS_HEIGHT * 0.5f, stripe_thickness);
        glPopMatrix();

        // 左右条带
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

        for (int i = 0; i < MAX_PARTICLES; i++) {
            if (!waterParticles[i].active) {
                waterParticles[i].active = true;
                waterParticles[i].life = 1.5f;
                waterParticles[i].x = static_cast<float>(nozzlePos[0]);
                waterParticles[i].y = static_cast<float>(nozzlePos[1]);
                waterParticles[i].z = static_cast<float>(nozzlePos[2]);

                waterParticles[i].vx = distribution(generator);
                waterParticles[i].vy = 1.0f;
                waterParticles[i].vz = distribution(generator);

                break;
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

void setupCamera() {
    if (g_isRobotView) {
        float robotGroundY = (CENTRAL_DISC_HEIGHT / 2.0f) + ROBOT_WHEEL_RADIUS;
        float angleRad = g_robot.angleY * M_PI / 180.0f;
        float localOffsetY = (ROBOT_CHASSIS_HEIGHT / 2.0f) + ROBOT_CAMERA_Y_OFFSET;
        float localOffsetZ = ROBOT_CAMERA_RADIUS + 0.1f;

        float worldOffsetX = sin(angleRad) * localOffsetZ;
        float worldOffsetZ = cos(angleRad) * localOffsetZ;

        vec3 eye;
        eye.x = g_robot.posX + worldOffsetX;
        eye.y = robotGroundY + localOffsetY;
        eye.z = g_robot.posZ + worldOffsetZ;

        vec3 lookAt;
        lookAt.x = eye.x + sin(angleRad) * 5.0f;
        lookAt.y = eye.y;
        lookAt.z = eye.z + cos(angleRad) * 5.0f;

        gluLookAt(eye.x, eye.y, eye.z,
            lookAt.x, lookAt.y, lookAt.z,
            0.0, 1.0, 0.0);

    }
    else {
        // Global view with rotation and zoom
        gluLookAt(0.0, 8.0 * g_zoomFactor, 20.0 * g_zoomFactor, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        glRotatef(g_cameraAngleX, 1.0f, 0.0f, 0.0f);
        glRotatef(g_cameraAngleY, 0.0f, 1.0f, 0.0f);
    }
}

// ==========================================================
// SCENE RENDERING
// ==========================================================

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

// Draw animated skimmer with orientation
void drawAnimatedSkimmer(float progress, const std::vector<vec3>& path) {
    vec3 currentPos = getPointOnPath(progress, path);
    float next_progress = progress + 0.01f;
    if (next_progress >= path.size()) next_progress -= path.size();
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

// Main display function
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    setupCamera();

    setupLights();

    drawSkyDome();
    // Central disc and its contents (hierarchical modeling)
    glPushMatrix();
    {
        glTranslatef(0.0f, CENTRAL_DISC_Y_POS, 0.0f);

        // Central disc platform
        glPushMatrix();
        glColor3f(0.3f, 0.6f, 0.2f);
        drawFloatingDisc(CENTRAL_DISC_RADIUS, CENTRAL_DISC_HEIGHT, g_texGroundCenter);
        glPopMatrix();

        // Robot on central disc
        glPushMatrix();
        {
            float robotGroundY = (CENTRAL_DISC_HEIGHT / 2.0f) + ROBOT_WHEEL_RADIUS;
            glTranslatef(g_robot.posX, robotGroundY, g_robot.posZ);
            drawRobot();
        }
        glPopMatrix();

        // Garden scene on disc surface
        glPushMatrix();
        {
            glTranslatef(0.0f, CENTRAL_DISC_HEIGHT / 2.0f, 0.0f);
            drawGardenScene();
            drawFlowerGarden();
        }
        glPopMatrix();

        // Robotic arm on disc surface
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
                drawFuturisticBuilding(BUILDING_BASE, BUILDING_HEIGHT, 15);
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

    glutSwapBuffers();
}

// ==========================================================
// CALLBACK FUNCTIONS
// ==========================================================

void reshape(int width, int height) {
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
        if (g_zoomFactor > 2.0f) g_zoomFactor = 2.0f;
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
        g_isRobotView = !g_isRobotView;
    }
    else if (key == 'p' || key == 'P') {
        isWatering = true;
    }
    else if (key == 't' || key == 'T') {
        g_showFlightPath = !g_showFlightPath;
    }
    // Robot light control
    else if (key == 'n' || key == 'N') {
        g_envLightOn = !g_envLightOn; 
        g_currentSkyIndex = (g_currentSkyIndex + 1) % 2; 
    }
    else if (key == '+' || key == '=') {
        g_robotLightBrightness += 0.1f;
        if (g_robotLightBrightness > 3.0f) g_robotLightBrightness = 1.0f;
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

void initGL() {
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    /*glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);*/
    // Initialize robot position
    float groundLevel = CENTRAL_DISC_Y_POS + (CENTRAL_DISC_HEIGHT / 2.0f);
    g_robot.posY = groundLevel + ROBOT_WHEEL_RADIUS;
    g_robot.posX = 1.0f;
    g_robot.posZ = 1.0f;
    g_robot.angleY = 0.0f;
    g_robot.wheelRotation = 0.0f;

    initParticles();
}

void initTextures() {
    g_texTreeBark = loadTexture("bark.bmp");
    g_texTreeLeaf = loadTexture("leaf.bmp");
    g_texGroundCenter = loadTexture("grass.bmp");
    g_texGroundSurround = loadTexture("stone.bmp");
    g_texBush = loadTexture("bush.bmp");
    g_texSkyDay = loadTexture("day.bmp");
    g_texSkyNight = loadTexture("night.bmp");
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Future City");

    initGL();
    initPaths();
    generateFractalTreeGrammar();
    initTextures();


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