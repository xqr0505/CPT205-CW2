#define FREEGLUT_STATIC
#include <GL/freeglut.h>
#include <cmath> 
#include <iostream> 
#include <vector>
#define M_PI 3.1415926535

struct vec3 { float x, y, z; };

// 向量辅助函数
vec3 vec3_add(vec3 a, vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
vec3 vec3_sub(vec3 a, vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
vec3 vec3_scale(vec3 v, float s) { return { v.x * s, v.y * s, v.z * s }; }
float vec3_length(vec3 v) { return sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
vec3 vec3_normalize(vec3 v) {
    float len = vec3_length(v);
    if (len > 0) return vec3_scale(v, 1.0f / len);
    return { 0, 0, 0 };
}
// --- 全局变量和常量 ---

// 摄像机/场景旋转角度
float g_cameraAngleY = 0.0f;

// 窗口尺寸
int g_windowWidth = 800;
int g_windowHeight = 600;
float g_cameraAngleX = 0.0f;  // 新增：上下旋转角度
float g_zoomFactor = 1.0f;    // 新增：缩放因子

// 鼠标状态
bool g_mouseLeftDown = false;
int g_mouseX, g_mouseY;

// 视角模式
bool g_isRobotView = false;

// --- 圆盘常量定义 ---
const float CENTRAL_DISC_RADIUS = 7.0f;
const float CENTRAL_DISC_HEIGHT = 0.5f;
const float CENTRAL_DISC_Y_POS = 0.0f;
const float SURROUND_DISC_RADIUS = 3.0f;
const float SURROUND_DISC_HEIGHT = 0.3f;
const float SURROUND_DISC_DISTANCE = 12.0f;
const float surround_heights[] = { 1.0f, -0.5f, 2.5f, -1.5f, 0.7f };


// --- 机器人常量定义 ---
const float ROBOT_BODY_WIDTH = 1.2f;  // X-axis
const float ROBOT_BODY_HEIGHT = 1.0f; // Y-axis
const float ROBOT_BODY_DEPTH = 0.8f;  // Z-axis
const float ROBOT_WHEEL_RADIUS = 0.4f;
const float ROBOT_WHEEL_THICKNESS = 0.2f;

const float ROBOT_MOVE_SPEED = 0.1f;
const float ROBOT_ROTATE_SPEED = 3.0f;


// --- 机器人状态结构体 ---
struct Robot {
    float posX, posY, posZ;
    float angleY;
    float wheelRotation;
};

Robot g_robot;

// ==========================================================
// 机械臂相关变量
// ==========================================================
// 机械臂
float armBaseAngle = 0.0f;
float armLowerAngle = 45.0f;
float armUpperAngle = 60.0f;
float nozzleAngle = 90.0f;
// 浇水动画的状态
bool isWatering = false;

const float ARM_BASE_HEIGHT = 0.5f;
const float ARM_BASE_RADIUS = 0.4f;
const float ARM_JOINT_RADIUS = 0.15f;
const float ARM_SEGMENT_LENGTH = 2.0f;
const float ARM_SEGMENT_WIDTH = 0.2f;
const float NOZZLE_RADIUS = 0.1f;
const float NOZZLE_LENGTH = 0.2f;

// ==========================================================
// 飞行器相关的全局变量和常量
// ==========================================================
const float WING_SPAN = 1.2f;          // 机翼从机身伸出的长度
const float WING_ROOT_CHORD = 0.8f;    // 翼根宽度 (连接机身处)
const float WING_TIP_CHORD = 0.4f;     // 翼尖宽度
const float WING_THICKNESS = 0.08f;    // 机翼厚度
const float STRIPE_THICKNESS = 0.04f;
const float SKIMMER_LENGTH = 1.5f;
const float SKIMMER_WIDTH = 0.5f;

// --- 路径和动画 ---
float g_skimmer1_progress = 0.0f;
float g_skimmer2_progress = 0.0f;
const float SKIMMER1_SPEED = 0.8f;  
const float SKIMMER2_SPEED = 0.65f;

// 存储路径控制点的容器
std::vector<vec3> g_skimmerPath1;
std::vector<vec3> g_skimmerPath2;


// ==========================================================
// 粒子系统相关
// ==========================================================
#define MAX_PARTICLES 1000
const float GRAVITY = 9.8f;

struct Particle {
    bool active;    // 粒子是否存活
    float life;     // 剩余生命周期
    float x, y, z;  // 位置
    float vx, vy, vz; // 速度
};

Particle waterParticles[MAX_PARTICLES];


// --- 函数声明 ---
void initGL();
void display();
void reshape(int width, int height);
void keyboard(unsigned char key, int x, int y);
void drawFloatingDisc(float radius, float height);
void keyboardUp(unsigned char key, int x, int y);

void mouse(int button, int state, int x, int y);
void motion(int x, int y);

// 机械臂相关函数
void drawArmBase();
void drawArmJoint();
void drawArmSegment();
void drawNozzle();
void drawWateringArm();

// 飞行器相关函数
void initPaths();

// 粒子系统相关函数
void initParticles();
void updateParticles(float dt);
void drawWaterParticles();

// 空闲函数
void idle();
void calculateNozzleWorldPosition(float baseRot, float lowerArmRot, float upperArmRot, GLdouble outPos[3]);

// --- 主函数 ---
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Floating Islands Scene with Robot");

    initGL();
    initPaths();
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


// --- 材质---

 
void setBuildingFrameMaterial() {
    glDisable(GL_COLOR_MATERIAL);
    GLfloat ambient[] = { 0.15f, 0.15f, 0.2f, 1.0f };
    GLfloat diffuse[] = { 0.2f, 0.2f, 0.25f, 1.0f };
    GLfloat specular[] = { 0.4f, 0.4f, 0.5f, 1.0f };
    GLfloat shininess = 30.0f;
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f }; 

    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);
}

void setGlowingWindowMaterial() {
    glDisable(GL_COLOR_MATERIAL);
    GLfloat black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    GLfloat emission[] = { 0.5f, 0.8f, 1.0f, 1.0f };

    glMaterialfv(GL_FRONT, GL_AMBIENT, black);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
    glMaterialfv(GL_FRONT, GL_SPECULAR, black);
    glMaterialf(GL_FRONT, GL_SHININESS, 0.0f);
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);

}
// --- 函数实现 ---

/**
 * @brief 初始化
 */
void initGL() {
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat light_pos[] = { 4.0f, 8.0f, 6.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    float groundLevel = CENTRAL_DISC_Y_POS + (CENTRAL_DISC_HEIGHT / 2.0f);
    g_robot.posY = groundLevel + ROBOT_WHEEL_RADIUS;
    g_robot.posX = 1.0f;
    g_robot.posZ = 1.0f;
    g_robot.angleY = 0.0f;
    g_robot.wheelRotation = 0.0f;

    // 初始化粒子系统
    initParticles();
}

/**
 * @brief 绘制悬浮圆盘
 */
void drawFloatingDisc(float radius, float height) {
    GLUquadric* quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);
    glPushMatrix();
    glTranslatef(0.0f, -height / 2.0f, 0.0f);
    glRotatef(90.0f, -1.0f, 0.0f, 0.0f);
    gluDisk(quadric, 0, radius, 32, 1);
    gluCylinder(quadric, radius, radius, height, 32, 5);
    glTranslatef(0.0f, 0.0f, height);
    gluDisk(quadric, 0, radius, 32, 1);
    glPopMatrix();
    gluDeleteQuadric(quadric);
}



/**
 * @brief 绘制灌木丛
 */
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


    glBegin(GL_TRIANGLES);
    for (int i = 0; i < 20; i++) {
        // 获取顶点
        const GLfloat* v1 = vertices[faces[i][0]];
        const GLfloat* v2 = vertices[faces[i][1]];
        const GLfloat* v3 = vertices[faces[i][2]];

        // 直接绘制，无需判断
        glNormal3fv(v1); glVertex3fv(v1);
        glNormal3fv(v2); glVertex3fv(v2);
        glNormal3fv(v3); glVertex3fv(v3);
    }
    glEnd();
}

/**
 * @brief 绘制花园场景
 */
void drawGardenScene() {
    GLdouble planeEquation[4] = { 0.0, 1.0, 0.0, 0.0 };
    glColor3f(0.2f, 0.6f, 0.2f);
    // 开启裁剪平面 
    glClipPlane(GL_CLIP_PLANE0, planeEquation);
    glEnable(GL_CLIP_PLANE0);

    // 绘制第一个灌木
    glPushMatrix();
    glTranslatef(-2.0f, 0.0f, 3.0f);
    glScalef(1.5f, 1.5f, 1.5f);
    drawBush();
    glPopMatrix();

    // 绘制第二个灌木
    glPushMatrix();
    glTranslatef(-3.5f, 0.0f, 2.0f);
    glScalef(1.2f, 1.2f, 1.2f);
    drawBush();
    glPopMatrix();

    // 绘制第三个灌木
    glPushMatrix();
    glTranslatef(-2.5f, 0.0f, 1.0f);
    glScalef(0.8f, 0.6f, 0.8f);
    drawBush();
    glPopMatrix();

    glDisable(GL_CLIP_PLANE0);
}

/**
 * @brief 绘制一片单独的、带有弧度的花瓣。
 * @param v_center 花瓣汇集的中心顶点
 * @param v_edge1 花瓣外边缘的第一个顶点
 * @param v_edge2 花瓣外边缘的第二个顶点
 */
void drawFlowerPetal(vec3 v_center, vec3 v_edge1, vec3 v_edge2) {
    const int ARC_SEGMENTS = 12;    
    const float ARC_HEIGHT = 0.5f;  

    // 1. 绘制花瓣的基底三角形
    glBegin(GL_TRIANGLES);
    glNormal3f(v_center.x, v_center.y, v_center.z); glVertex3f(v_center.x, v_center.y, v_center.z);
    glNormal3f(v_edge1.x, v_edge1.y, v_edge1.z); glVertex3f(v_edge1.x, v_edge1.y, v_edge1.z);
    glNormal3f(v_edge2.x, v_edge2.y, v_edge2.z); glVertex3f(v_edge2.x, v_edge2.y, v_edge2.z);
    glEnd();

    // 2. 绘制弯曲的瓣面
    vec3 mid_point = vec3_scale(vec3_add(v_edge1, v_edge2), 0.5f);
    vec3 arc_direction = vec3_normalize(vec3_sub(mid_point, v_center));

    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= ARC_SEGMENTS; ++i) {
        float step = (float)i / ARC_SEGMENTS;

        vec3 edge_point = vec3_add(v_edge1, vec3_scale(vec3_sub(v_edge2, v_edge1), step));
        float arc_offset_magnitude = sin(step * M_PI) * ARC_HEIGHT;
        vec3 arc_point = vec3_add(edge_point, vec3_scale(arc_direction, arc_offset_magnitude));

        vec3 normal1 = vec3_normalize(edge_point);
        glNormal3f(normal1.x, normal1.y, normal1.z);
        glVertex3f(edge_point.x, edge_point.y, edge_point.z);

        vec3 normal2 = vec3_normalize(arc_point);
        glNormal3f(normal2.x, normal2.y, normal2.z);
        glVertex3f(arc_point.x, arc_point.y, arc_point.z);
    }
    glEnd();
}


void drawFlower(vec3 petalColor) {

    glPushMatrix();

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
        const GLfloat* vC_ptr = vertices[faces[i][0]];
        const GLfloat* vA_ptr = vertices[faces[i][1]];
        const GLfloat* vB_ptr = vertices[faces[i][2]];
        vec3 vC = { vC_ptr[0], vC_ptr[1], vC_ptr[2] };
        vec3 vA = { vA_ptr[0], vA_ptr[1], vA_ptr[2] };
        vec3 vB = { vB_ptr[0], vB_ptr[1], vB_ptr[2] };

        drawFlowerPetal(vC, vA, vB);
    }
    glPopMatrix();
}
void drawFlowerGarden() {

    glPushMatrix();
    glTranslatef(-0.6f, 0.5f, 2.0f); 
    glRotatef(120.0f, 1.0f, 0.0f, 0.0f);
    glRotatef(20.0f, 0.0f, 0.0f, 1.0f);
    glScalef(0.4f, 0.4f, 0.4f);     
    drawFlower({ 1.0f, 0.6f, 0.8f }); 
    glPopMatrix();


    glPushMatrix();
    glTranslatef(-1.6f, 0.6f, 1.4f);
    glRotatef(120.0f, 1.0f, 0.0f, 0.0f);
    glRotatef(20.0f, 0.0f, 0.0f, 1.0f);
    glScalef(0.3f, 0.3f, 0.3f);
    drawFlower({ 1.0f, 0.6f, 0.8f });
    glPopMatrix();
}


// ==========================================================
// 机械臂组件绘制函数
// ==========================================================

void drawArmBase() {
    glPushMatrix();
    glColor3f(0.2f, 0.2f, 0.2f);
    glScalef(ARM_BASE_RADIUS, ARM_BASE_HEIGHT, ARM_BASE_RADIUS);
    glutSolidCube(1.0f);
    glPopMatrix();
}

void drawArmJoint() {
    glColor3f(0.2f, 0.2f, 0.2f); // 黑色金属材质
    glutSolidSphere(ARM_JOINT_RADIUS, 20, 20);
}

void drawArmSegment() {
    glPushMatrix();
    glColor3f(0.75f, 0.75f, 0.75f); // 银色材质
    glScalef(ARM_SEGMENT_LENGTH, ARM_SEGMENT_WIDTH, ARM_SEGMENT_WIDTH);
    glutSolidCube(1.0f);
    glPopMatrix();
}

void drawNozzle() {
    glPushMatrix();
    glColor3f(0.75f, 0.75f, 0.75f); // 银色材质
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, NOZZLE_RADIUS, NOZZLE_RADIUS, NOZZLE_LENGTH, 20, 20);
    gluDeleteQuadric(quad);
    glPopMatrix();
}

/**
 * @brief 绘制一栋带有发光窗户的未来风格建筑。
 * @param baseSize 建筑底座的边长
 * @param height 建筑的高度
 * @param numWindowFloors 窗户的层数
 */
void drawFuturisticBuilding(float baseSize, float height, int numWindowFloors) {

    // --- 1. 绘制深灰色的建筑主体框架 ---
    setBuildingFrameMaterial();
    glPushMatrix();
    glTranslatef(0.0f, height / 2.0f, 0.0f); 
    glScalef(baseSize, height, baseSize);
    glutSolidCube(1.0f);
    glPopMatrix();

    // --- 2. 绘制发光的蓝色窗户 ---
    setGlowingWindowMaterial();

    float windowHeight = height / (float)numWindowFloors * 0.6f; 
    float floorHeight = height / (float)numWindowFloors;         
    float windowDepthOffset = baseSize * 0.505f;                

    for (int i = 0; i < numWindowFloors; ++i) {
        float y_pos = i * floorHeight + floorHeight * 0.2f; 

        // 绘制四面的窗户条
        for (int side = 0; side < 4; ++side) {
            glPushMatrix();
            glRotatef(90.0f * side, 0.0f, 1.0f, 0.0f);

            glTranslatef(0.0f, y_pos, windowDepthOffset);

            glScalef(baseSize * 0.8f, windowHeight, 0.01f); 
            glutSolidCube(1.0f);
            glPopMatrix();
        }
    }

    glEnable(GL_COLOR_MATERIAL);
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);
}

// ==========================================================
// 主机械臂绘制函数
// ==========================================================
void drawWateringArm() {
    glPushMatrix(); 
    {
        // 整个机械臂的根位置
        glTranslatef(0.0f, 0.1f, 0.0f);

        // 1. 绘制底座并应用底座的旋转
        glRotatef(armBaseAngle, 0.0f, 1.0f, 0.0f);
        drawArmBase();

        // 2. 变换到下臂
        glPushMatrix(); // 保存底座的矩阵状态
        {
            glTranslatef(0.0f, ARM_BASE_HEIGHT, 0.0f); 
            glRotatef(armLowerAngle, 0.0f, 0.0f, 1.0f); 
            drawArmJoint();

            // 绘制下臂臂干
            glPushMatrix();
            glTranslatef(ARM_SEGMENT_LENGTH / 2.0f, 0.0f, 0.0f);
            drawArmSegment();
            glPopMatrix();

            // 3. 移动到第二个关节的位置（即上臂的起点）
            glTranslatef(ARM_SEGMENT_LENGTH, 0.0f, 0.0f);
            glRotatef(armUpperAngle, 0.0f, 0.0f, 1.0f); 
            drawArmJoint();

            // 绘制上臂臂干
            glPushMatrix();
            glTranslatef(ARM_SEGMENT_LENGTH / 2.0f, 0.0f, 0.0f);
            drawArmSegment();
            glPopMatrix();

            // 4. 移动到喷头的位置（即上臂的末端）
            glTranslatef(ARM_SEGMENT_LENGTH, 0.0f, 0.0f);
            glRotatef(nozzleAngle, 0.0f, 0.0f, 1.0f); // 喷头绕Z轴旋转
            drawNozzle();

        }
        glPopMatrix(); // 恢复到底座的矩阵状态
    }
    glPopMatrix(); 
}

// ==========================================================
// B. 添加飞行器的材质和绘制函数
// ==========================================================



/**
 * @brief 设置发光的蓝色条带材质。
 */
void setGlowingStripeMaterial() {
    glDisable(GL_COLOR_MATERIAL);
    GLfloat black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    GLfloat emission[] = { 0.5f, 0.8f, 1.0f, 1.0f }; 

    glMaterialfv(GL_FRONT, GL_AMBIENT, black);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
    glMaterialfv(GL_FRONT, GL_SPECULAR, black);
    glMaterialf(GL_FRONT, GL_SHININESS, 0.0f);
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);
}



/**
 * @brief 绘制一个从原点沿X轴正方向伸出的机翼。
 */
void drawWing() {
    glBegin(GL_QUADS);

    // 上表面
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);

    // 下表面
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);

    // 翼尖
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);

    // 后缘
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, WING_TIP_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);

    // 前缘
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);
    glVertex3f(WING_SPAN, -WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);
    glVertex3f(WING_SPAN, WING_THICKNESS / 2.0f, -WING_TIP_CHORD / 2.0f);

    // 翼根
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, -WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);
    glVertex3f(0.0f, WING_THICKNESS / 2.0f, -WING_ROOT_CHORD / 2.0f);


    glEnd();
}

/**
 * @brief 在机身特定 Z 轴位置绘制一个发光的环。
 * @param z_position 环的中心 Z 坐标。
 * @param thickness 环的厚度 (沿 Z 轴)。
 */
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

        vec3 normal = { x, y, 0.0f }; 
        normal = vec3_normalize(normal);

        // 后顶点
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(x * ring_radius, y * ring_radius, z_position - thickness / 2.0f);

        // 前顶点
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(x * ring_radius, y * ring_radius, z_position + thickness / 2.0f);
    }
    glEnd();
}

/**
 * @brief 绘制一个由机身、机翼和发光条带组成的、重新设计的飞行器。
 */
void drawSkimmer() {
    // 机身 

    glPushMatrix();
    glColor3f(0.2f, 0.2f, 0.2f);
    glScalef(SKIMMER_WIDTH, SKIMMER_WIDTH, SKIMMER_LENGTH); 
    glutSolidSphere(1.0, 16, 12);
    glPopMatrix();

    // 机翼 
    // 右翼
    glPushMatrix();
    {
        glTranslatef(SKIMMER_WIDTH * 0.5f, 0.0f, 0.0f);
        glRotatef(5.0f, 0.0f, 0.0f, 1.0f);
        drawWing();
    }
    glPopMatrix();

    // 左翼
    glPushMatrix();
    {
        glTranslatef(-SKIMMER_WIDTH * 0.5f, 0.0f, 0.0f);
        glRotatef(5.0f, 0.0f, 0.0f, 1.0f);
        glScalef(-1.0f, 1.0f, 1.0f);

        glFrontFace(GL_CW); 
        drawWing();
        glFrontFace(GL_CCW); 
    }
    glPopMatrix();
    setGlowingStripeMaterial();
    const float stripe_thickness = 0.2f;

    // 第一个环
    drawGlowingRing(SKIMMER_LENGTH * 0.3f, stripe_thickness);

    // 第二个环
    drawGlowingRing(SKIMMER_LENGTH * -0.3f, stripe_thickness);


    glEnable(GL_COLOR_MATERIAL);
    GLfloat emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);


}
// ==========================================================
// C. 添加路径计算函数
// ==========================================================

/**
 * @brief 在Catmull-Rom样条曲线的单个线段上进行插值。
 * @param p0, p1, p2, p3 四个控制点，曲线在 p1 和 p2 之间生成。
 * @param t 插值因子，范围 [0, 1]。
 * @return 返回在曲线上的插值点。
 */
vec3 getCatmullRomPoint(vec3 p0, vec3 p1, vec3 p2, vec3 p3, float t) {
    vec3 result;
    float t2 = t * t;
    float t3 = t2 * t;

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

/**
 * @brief 从一个完整的点集中获取样条曲线上的点。
 * @param progress 沿整个路径的进度 (例如 2.5 表示在第2个线段的中点)。
 * @param path 控制点的向量。
 * @return 返回在完整路径上的插值点。
 */
vec3 getPointOnPath(float progress, const std::vector<vec3>& path) {
    if (path.size() < 4) return { 0, 0, 0 }; 

    int numPoints = path.size();
    int p1_idx = (int)progress; 
    float t = progress - p1_idx;      

    int p0_idx = (p1_idx - 1 + numPoints) % numPoints;
    int p2_idx = (p1_idx + 1) % numPoints;
    int p3_idx = (p1_idx + 2) % numPoints;

    return getCatmullRomPoint(path[p0_idx], path[p1_idx], path[p2_idx], path[p3_idx], t);
}

void initPaths() {
    float r = SURROUND_DISC_DISTANCE;
    // --- 路径 1 ---
    g_skimmerPath1.push_back({ r + 8.0f, 3.0f, 0.0f });      
    g_skimmerPath1.push_back({ 0.0f, 6.0f, r + 6.0f }); 
    g_skimmerPath1.push_back({ -r + 7.0f, 4.0f, r - 8.0f });
    g_skimmerPath1.push_back({ -r - 4.0f, 5.0f, 0.0f });    
    g_skimmerPath1.push_back({ -r, 6.0f, -r + 2.0f });
    g_skimmerPath1.push_back({ 0.0f, 10.0f, -r });


    // --- 路径 2 ---
    g_skimmerPath2.push_back({ r - 7.0f, 3.0f, 0.0f });
    g_skimmerPath2.push_back({ 0.0f, 6.0f, r + 3.0f });
    g_skimmerPath2.push_back({ -r - 8.0f, 4.0f, r - 8.0f });
    g_skimmerPath2.push_back({ -r - 4.0f, 6.0f, 0.0f });
    g_skimmerPath2.push_back({ -r, 6.0f, -r + 1.0f });
    g_skimmerPath2.push_back({ 1.0f, 3.0f, -r - 5.0f });
}
// ==========================================================
// 粒子系统函数
// ==========================================================

// 计算机械臂喷头的世界坐标位置
void calculateNozzleWorldPosition(float baseRot, float lowerArmRot, float upperArmRot, GLdouble outPos[3]) {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // 1. 考虑中央圆盘的位置
    glTranslatef(0.0f, CENTRAL_DISC_Y_POS, 0.0f);
    
    // 2. 机械臂在圆盘表面
    glTranslatef(0.0f, CENTRAL_DISC_HEIGHT / 2.0f, 0.0f);

    // 3. 机械臂基座位置
    glTranslatef(0.0f, 0.1f, 0.0f);

    // 4. 底座旋转
    glRotatef(baseRot, 0.0f, 1.0f, 0.0f);

    // 5. 移动到第一个关节位置
    glTranslatef(0.0f, ARM_BASE_HEIGHT, 0.0f);

    // 6. 下臂旋转
    glRotatef(lowerArmRot, 0.0f, 0.0f, 1.0f);

    // 7. 移动到第二个关节位置
    glTranslatef(ARM_SEGMENT_LENGTH, 0.0f, 0.0f);

    // 8. 上臂旋转
    glRotatef(upperArmRot, 0.0f, 0.0f, 1.0f);

    // 9. 移动到喷头末端
    glTranslatef(ARM_SEGMENT_LENGTH, 0.0f, 0.0f);

    // 10. 喷头旋转
    glRotatef(nozzleAngle, 0.0f, 0.0f, 1.0f);

    // 11. 移动到喷头出水口位置
    glTranslatef(NOZZLE_LENGTH, 0.0f, 0.0f);

    // 获取当前变换矩阵
    GLdouble matrix[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, matrix);

    // 提取位置信息
    outPos[0] = matrix[12];
    outPos[1] = matrix[13];
    outPos[2] = matrix[14];

    glPopMatrix();
}

void initParticles() {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        waterParticles[i].active = false;
    }
}

void updateParticles(float dt) {
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

                waterParticles[i].vx = (rand() % 100 / 100.0f - 0.5f) * 0.5f;
                waterParticles[i].vy = -2.0f - (rand() % 100 / 100.0f);
                waterParticles[i].vz = (rand() % 100 / 100.0f - 0.5f) * 0.5f;
                break;
            }
        }
    }

    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (waterParticles[i].active) {
            waterParticles[i].x += waterParticles[i].vx * dt;
            waterParticles[i].y += waterParticles[i].vy * dt;
            waterParticles[i].z += waterParticles[i].vz * dt;
            waterParticles[i].vy -= GRAVITY * dt;

            waterParticles[i].life -= dt;
            if (waterParticles[i].life <= 0.0f || waterParticles[i].y < (CENTRAL_DISC_Y_POS + CENTRAL_DISC_HEIGHT / 2.0f)) { // 落地或生命结束
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

/**
 * @brief 渲染场景 - 使用层次化建模
 */
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // --- 设置相机 ---
    if (g_isRobotView) {
        float angleRad = g_robot.angleY * M_PI / 180.0f;

        float camX = g_robot.posX - sin(angleRad) * 4.0f;
        float camY = g_robot.posY + 2.0f;
        float camZ = g_robot.posZ - cos(angleRad) * 4.0f;

        gluLookAt(camX, camY, camZ,
            g_robot.posX, g_robot.posY, g_robot.posZ,
            0.0, 1.0, 0.0);

    }
    else {
        // 全局旋转视角，应用鼠标旋转和缩放
        gluLookAt(0.0, 8.0 * g_zoomFactor, 20.0 * g_zoomFactor, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        glRotatef(g_cameraAngleX, 1.0f, 0.0f, 0.0f);  // 上下旋转
        glRotatef(g_cameraAngleY, 0.0f, 1.0f, 0.0f);  // 左右旋转
    }

    // ==========================================================
    // 层次化建模
    // ==========================================================
    

    glPushMatrix(); 
    {
        glTranslatef(0.0f, CENTRAL_DISC_Y_POS, 0.0f);
        
        // 绘制中央圆盘
        glPushMatrix();
        glColor3f(0.3f, 0.6f, 0.2f);
        drawFloatingDisc(CENTRAL_DISC_RADIUS, CENTRAL_DISC_HEIGHT);
        glPopMatrix();

        glPushMatrix();
        {
            float robotLocalY = (CENTRAL_DISC_HEIGHT / 2.0f) + ROBOT_WHEEL_RADIUS;
            glTranslatef(g_robot.posX, robotLocalY, g_robot.posZ);
            glRotatef(g_robot.angleY, 0.0f, 1.0f, 0.0f);

            // 绘制机器人身体
            glPushMatrix();
            glColor3f(0.8f, 0.2f, 0.2f);
            glScalef(ROBOT_BODY_WIDTH, ROBOT_BODY_HEIGHT, ROBOT_BODY_DEPTH);
            glutSolidCube(1.0f);
            glPopMatrix();

            // 绘制左轮
            glPushMatrix();
            glColor3f(0.3f, 0.3f, 0.3f);
            glTranslatef(-ROBOT_BODY_WIDTH / 2.0f - ROBOT_WHEEL_THICKNESS / 2.0f, 0.0f, 0.0f);
            glRotatef(90.0, 0.0, 0.0, 1.0);
            glRotatef(g_robot.wheelRotation, 0.0f, 1.0f, 0.0f);
            drawFloatingDisc(ROBOT_WHEEL_RADIUS, ROBOT_WHEEL_THICKNESS);
            glPopMatrix();

            // 绘制右轮
            glPushMatrix();
            glColor3f(0.3f, 0.3f, 0.3f);
            glTranslatef(ROBOT_BODY_WIDTH / 2.0f + ROBOT_WHEEL_THICKNESS / 2.0f, 0.0f, 0.0f);
            glRotatef(90.0, 0.0, 0.0, 1.0);
            glRotatef(g_robot.wheelRotation, 0.0f, 1.0f, 0.0f);
            drawFloatingDisc(ROBOT_WHEEL_RADIUS, ROBOT_WHEEL_THICKNESS);
            glPopMatrix();
        }
        glPopMatrix();

        glPushMatrix();
        {
            glTranslatef(0.0f, CENTRAL_DISC_HEIGHT / 2.0f, 0.0f);
            drawGardenScene();
            drawFlowerGarden();
        }
        glPopMatrix();

        glPushMatrix();
        {
            glTranslatef(0.0f, CENTRAL_DISC_HEIGHT / 2.0f, 0.0f);
            drawWateringArm();
        }
        glPopMatrix();
    }
    glPopMatrix(); 

    const float BUILDING_HEIGHT = 6.5f; 
    const float BUILDING_BASE = 3.5f;  

    for (int i = 0; i < 5; ++i) {
        glPushMatrix(); 
        {

            float angle = i * 72.0f;
            glRotatef(angle, 0.0f, 1.0f, 0.0f);
            glTranslatef(0.0f, surround_heights[i], -SURROUND_DISC_DISTANCE);

            glColor3f(0.5f, 0.5f, 0.5f);
            drawFloatingDisc(SURROUND_DISC_RADIUS, SURROUND_DISC_HEIGHT);

            glPushMatrix();
            {

                glTranslatef(0.0f, SURROUND_DISC_HEIGHT / 2.0f, 0.0f);


                drawFuturisticBuilding(BUILDING_BASE, BUILDING_HEIGHT, 15); 
            }
            glPopMatrix();
        }
        glPopMatrix(); 
    }

    // 6. 绘制水粒子
    drawWaterParticles();

    // 7. 绘制飞行器
    glDisable(GL_LIGHTING);  
    glLineWidth(2.0f);  
    glColor3f(0.5f, 0.6f, 0.9f);
    // 绘制路径1
    if (!g_skimmerPath1.empty()) {
        glBegin(GL_LINE_STRIP);
        for (float t = 0.0f; t < g_skimmerPath1.size(); t += 0.1f) {  
            vec3 point = getPointOnPath(t, g_skimmerPath1);
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
    }
    glColor3f(0.95f, 0.4f, 0.1f);
    // 绘制路径2
    if (!g_skimmerPath2.empty()) {
        glBegin(GL_LINE_STRIP);
        for (float t = 0.0f; t < g_skimmerPath2.size(); t += 0.1f) { 
            vec3 point = getPointOnPath(t, g_skimmerPath2);
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
    }
    glEnable(GL_LIGHTING);  
    glLineWidth(1.0f);  

    vec3 currentPos, nextPos, direction;
    float next_progress;

    // --- 绘制第一个飞行器 ---
    currentPos = getPointOnPath(g_skimmer1_progress, g_skimmerPath1);
    next_progress = g_skimmer1_progress + 0.01f; 
    if (next_progress >= g_skimmerPath1.size()) next_progress -= g_skimmerPath1.size();
    nextPos = getPointOnPath(next_progress, g_skimmerPath1);
    direction = vec3_normalize(vec3_sub(nextPos, currentPos));

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


    // --- 绘制第二个飞行器 ---
    currentPos = getPointOnPath(g_skimmer2_progress, g_skimmerPath2);
    next_progress = g_skimmer2_progress + 0.01f; 
    if (next_progress >= g_skimmerPath2.size()) next_progress -= g_skimmerPath2.size();
    nextPos = getPointOnPath(next_progress, g_skimmerPath2);
    direction = vec3_normalize(vec3_sub(nextPos, currentPos));

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
    glutSwapBuffers();
}

/**
 * @brief 窗口尺寸变化回调
 */
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
        if (state == GLUT_DOWN) {
            g_mouseLeftDown = true;
            g_mouseX = x;
            g_mouseY = y;
        }
        else if (state == GLUT_UP) {
            g_mouseLeftDown = false;
        }
    }
    else if (button == 3) {  // 滚轮向上，缩小
        g_zoomFactor *= 0.9f;
        if (g_zoomFactor < 0.5f) g_zoomFactor = 0.5f;
        glutPostRedisplay();
    }
    else if (button == 4) {  // 滚轮向下，放大
        g_zoomFactor *= 1.1f;
        if (g_zoomFactor > 3.0f) g_zoomFactor = 3.0f;
        glutPostRedisplay();
    }
}


void motion(int x, int y) {
    if (g_mouseLeftDown && !g_isRobotView) {  // 只在非机器人视角下生效
        int dx = x - g_mouseX;
        int dy = y - g_mouseY;
        g_cameraAngleY += dx * 0.5f;  // 左右旋转
        g_cameraAngleX += dy * 0.5f;  // 上下旋转
        // 限制上下旋转角度，避免翻转
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

    // 保存旧角度
    float oldLowerAngle = armLowerAngle;
    float oldUpperAngle = armUpperAngle;
    float oldBaseAngle = armBaseAngle;

    if (key == 'w' || key == 'W') { // 前进
        nextX += sin(angleRad) * ROBOT_MOVE_SPEED;
        nextZ += cos(angleRad) * ROBOT_MOVE_SPEED;
        g_robot.wheelRotation -= 15.0f;
    }
    else if (key == 's' || key == 'S') { // 后退
        nextX -= sin(angleRad) * ROBOT_MOVE_SPEED;
        nextZ -= cos(angleRad) * ROBOT_MOVE_SPEED;
        g_robot.wheelRotation += 15.0f;
    }
    else if (key == 'a' || key == 'A') { // 左转
        g_robot.angleY += ROBOT_ROTATE_SPEED;
    }
    else if (key == 'd' || key == 'D') { // 右转
        g_robot.angleY -= ROBOT_ROTATE_SPEED;
    }
    else if (key == 'c' || key == 'C') { // 切换视角
        g_isRobotView = !g_isRobotView;
    }
    else if (key == 'p' || key == 'P') { // 按下 p 键开始浇水
        isWatering = true;
    }
    else if (key == '1') { // 大臂向上
        armLowerAngle += 5.0f;
    }
    else if (key == '2') { // 大臂向下
        armLowerAngle -= 5.0f;
    }
    else if (key == '3') { // 小臂向上
        armUpperAngle += 5.0f;
    }
    else if (key == '4') { // 小臂向下
        armUpperAngle -= 5.0f;
    }
    else if (key == '5') { // 底座逆时针旋转
        armBaseAngle += 5.0f;
    }
    else if (key == '6') { // 底座顺时针旋转
        armBaseAngle -= 5.0f;
    }

    // 检查机械臂喷头是否低于中心圆盘表面
    GLdouble nozzlePos[3];
    calculateNozzleWorldPosition(armBaseAngle, armLowerAngle, armUpperAngle, nozzlePos);
    float groundLevel = CENTRAL_DISC_Y_POS + CENTRAL_DISC_HEIGHT / 2.0f;
    if (nozzlePos[1] < groundLevel + 0.1f) {
        // 恢复旧角度，阻止本次移动
        armLowerAngle = oldLowerAngle;
        armUpperAngle = oldUpperAngle;
        armBaseAngle = oldBaseAngle;
    }

    // 移动机器人时的边界检查
    float distanceFromCenter = sqrt(nextX * nextX + nextZ * nextZ);
    if (distanceFromCenter < CENTRAL_DISC_RADIUS - (ROBOT_BODY_WIDTH / 2.0f)) {
        g_robot.posX = nextX;
        g_robot.posZ = nextZ;
    }

    glutPostRedisplay();
}

void keyboardUp(unsigned char key, int x, int y) {
    if (key == 'p' || key == 'P') { // 松开 p 键停止浇水
        isWatering = false;
    }
}

/**
 * @brief 更新粒子系统
 */
void idle() {
    static int lastTime = glutGet(GLUT_ELAPSED_TIME);
    int currentTime = glutGet(GLUT_ELAPSED_TIME);
    float dt = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;

    updateParticles(dt);

    // 更新飞行器1的进度，并处理循环
    g_skimmer1_progress += SKIMMER1_SPEED * dt;
    if (g_skimmer1_progress >= g_skimmerPath1.size()) {
        g_skimmer1_progress -= g_skimmerPath1.size();
    }

    // 更新飞行器2的进度，并处理循环
    g_skimmer2_progress += SKIMMER2_SPEED * dt;
    if (g_skimmer2_progress >= g_skimmerPath2.size()) {
        g_skimmer2_progress -= g_skimmerPath2.size();
    }

    glutPostRedisplay();
}