#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

// BEGIN MATRIX MATH

// Constants
#define X 0
#define Y 1
#define Z 2
#define GRAVITY 9.81

// Structure to hold a 3x3 matrix
typedef struct
{
   double data[3][3];
} Matrix3x3;

// Structure to hold a 3D vector
typedef struct
{
   double x;
   double y;
   double z;
} Vector3;

// Function prototypes
Matrix3x3 create_rotation_matrix(double accel_x, double accel_y, double accel_z);
double get_inclination_angle(Matrix3x3 rotation_matrix);
Vector3 cross_product(Vector3 a, Vector3 b);
double vector_magnitude(Vector3 v);
Vector3 normalize_vector(Vector3 v);
double dot_product(Vector3 a, Vector3 b);

// Helper function to create a Vector3
Vector3 create_vector3(double x, double y, double z)
{
   Vector3 v = {x, y, z};
   return v;
}

// Create identity matrix
Matrix3x3 create_identity_matrix(void)
{
   Matrix3x3 m = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
   return m;
}

Matrix3x3 create_rotation_matrix(double accel_x, double accel_y, double accel_z)
{
   double acc_magnitude = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

   // Avoid division by zero
   if (acc_magnitude < 1e-6)
   {
      return create_identity_matrix();
   }

   // Normalize acceleration vector
   Vector3 acc = create_vector3(
       accel_x / acc_magnitude,
       accel_y / acc_magnitude,
       accel_z / acc_magnitude);

   // Calculate rotation axis using cross product
   Vector3 up = create_vector3(0, 1, 0);
   Vector3 right = cross_product(acc, up);
   double right_magnitude = vector_magnitude(right);

   // Handle special cases
   if (right_magnitude < 1e-6)
   {
      if (acc.z > 0)
      {
         return create_identity_matrix();
      }
      else
      {
         Matrix3x3 m = {{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};
         return m;
      }
   }

   // Normalize right vector
   right = normalize_vector(right);

   // Calculate backward vector
   Vector3 backward = cross_product(acc, right);
   backward = normalize_vector(backward);

   // Create rotation matrix with columns in correct order
   Matrix3x3 K;
   // First column (right vector)
   K.data[0][0] = right.x;
   K.data[1][0] = right.y;
   K.data[2][0] = right.z;
   // Second column (backward vector)
   K.data[0][1] = backward.x;
   K.data[1][1] = backward.y;
   K.data[2][1] = backward.z;
   // Third column (acceleration vector)
   K.data[0][2] = acc.x;
   K.data[1][2] = acc.y;
   K.data[2][2] = acc.z;

   return K;
}

double get_inclination_angle(Matrix3x3 rotation_matrix)
{
   // Extract z-axis (third column) from rotation matrix
   Vector3 z_axis = create_vector3(
       rotation_matrix.data[0][2],
       0.0,
       rotation_matrix.data[2][2]);

   z_axis = normalize_vector(z_axis);
   Vector3 vertical = create_vector3(0, 0, 1);

   // Calculate angle using dot product
   double cos_angle = dot_product(z_axis, vertical);

   // Clamp to [-1, 1] to handle numerical errors
   if (cos_angle > 1.0)
      cos_angle = 1.0;
   if (cos_angle < -1.0)
      cos_angle = -1.0;

   return acos(cos_angle);
}

// Vector operations implementations
Vector3 cross_product(Vector3 a, Vector3 b)
{
   Vector3 result = {
       a.y * b.z - a.z * b.y,
       a.z * b.x - a.x * b.z,
       a.x * b.y - a.y * b.x};
   return result;
}

double vector_magnitude(Vector3 v)
{
   return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 normalize_vector(Vector3 v)
{
   double mag = vector_magnitude(v);
   if (mag < 1e-6)
   {
      return create_vector3(0, 0, 0);
   }
   return create_vector3(v.x / mag, v.y / mag, v.z / mag);
}

double dot_product(Vector3 a, Vector3 b)
{
   return a.x * b.x + a.y * b.y + a.z * b.z;
}

// END MATRIX MATH

// BEGIN OPENGL CODE

// Global variables for OpenGL visualization
float angle = 0.0f;
Matrix3x3 current_rotation;
int window_width = 800;
int window_height = 600;

// Global variables for OpenGL mouse control
float camera_distance = 5.0f;
float camera_theta = 45.0f; // Horizontal angle
float camera_phi = 35.264f; // Vertical angle (arctan(1/√2) for isometric view)
int mouse_last_x = 0;
int mouse_last_y = 0;
bool mouse_left_pressed = false;

// Globals for live acceleration vector visualization
double current_accel_x = 0.0;
double current_accel_y = 0.0;
double current_accel_z = GRAVITY;
double target_accel_x = 0.0;
double target_accel_y = 0.0;
double target_accel_z = GRAVITY;
int animation_time = 0;

// Render text in 3D space
void render_text_3d(float x, float y, float z, const char *text)
{
   glRasterPos3f(x, y, z);
   for (const char *c = text; *c != '\0'; c++)
   {
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
   }
}

// Draw text in screen space (for additional information)
void render_text_2d(float x, float y, const char *text)
{
   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
   glLoadIdentity();
   gluOrtho2D(0.0, window_width, 0.0, window_height);

   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glLoadIdentity();

   glColor3f(0.0f, 0.0f, 0.0f); // Black text
   glRasterPos2f(x, y);
   for (const char *c = text; *c != '\0'; c++)
   {
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
   }

   glPopMatrix();
   glMatrixMode(GL_PROJECTION);
   glPopMatrix();
   glMatrixMode(GL_MODELVIEW);
}

// Update draw_coordinate_axes with adjusted label positions
void draw_coordinate_axes(float length)
{
   glLineWidth(2.0f);

   // X axis (red)
   glColor3f(1.0f, 0.0f, 0.0f);
   glBegin(GL_LINES);
   glVertex3f(0.0f, 0.0f, 0.0f);
   glVertex3f(length, 0.0f, 0.0f);
   glEnd();
   render_text_3d(length + 0.15f, 0.0f, 0.0f, "X");

   // Y axis (green)
   glColor3f(0.0f, 1.0f, 0.0f);
   glBegin(GL_LINES);
   glVertex3f(0.0f, 0.0f, 0.0f);
   glVertex3f(0.0f, length, 0.0f);
   glEnd();
   render_text_3d(0.0f, length + 0.15f, 0.0f, "Y");

   // Z axis (blue)
   glColor3f(0.0f, 0.0f, 1.0f);
   glBegin(GL_LINES);
   glVertex3f(0.0f, 0.0f, 0.0f);
   glVertex3f(0.0f, 0.0f, length);
   glEnd();
   render_text_3d(0.0f, 0.0f, length + 0.15f, "Z");
}

void draw_rocket(void)
{
   // Main body color (white/light gray)
   glColor3f(0.9f, 0.9f, 0.9f);

   // Main body (cylinder)
   GLUquadricObj *quadric = gluNewQuadric();
   gluQuadricNormals(quadric, GLU_SMOOTH);

   glPushMatrix();
   glRotatef(90.0f, 0.0f, 1.0f, 0.0f); // Rotate to align with X axis

   // Main body cylinder
   gluCylinder(quadric, 0.3f, 0.3f, 2.0f, 32, 32);

   // Nose cone (red)
   glColor3f(0.8f, 0.2f, 0.2f);
   glPushMatrix();
   glTranslatef(0.0f, 0.0f, 2.0f);
   gluCylinder(quadric, 0.3f, 0.0f, 0.8f, 32, 32);
   glPopMatrix();

   // Base cap
   glColor3f(0.8f, 0.8f, 0.8f);
   glPushMatrix();
   glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
   gluDisk(quadric, 0.0f, 0.3f, 32, 32);
   glPopMatrix();

   // Engine nozzle (dark gray)
   glColor3f(0.3f, 0.3f, 0.3f);
   glPushMatrix();
   glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
   gluCylinder(quadric, 0.2f, 0.3f, 0.4f, 32, 32);
   // Engine interior (darker)
   glColor3f(0.1f, 0.1f, 0.1f);
   gluDisk(quadric, 0.0f, 0.2f, 32, 32);
   glPopMatrix();

   glPopMatrix();

   // Draw fins (4 of them)
   glColor3f(0.7f, 0.7f, 0.7f);
   for (int i = 0; i < 4; i++)
   {
      glPushMatrix();
      glRotatef(90.0f * i, 1.0f, 0.0f, 0.0f); // Rotate around rocket body
      glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);    // Rotate fin to correct orientation

      // Draw fin with curved edge
      glBegin(GL_TRIANGLE_FAN);
      // Center point at base
      glVertex3f(0.0f, 0.3f, 0.0f);

      // Curved outer edge
      for (float t = 0.0f; t <= 1.0f; t += 0.1f)
      {
         float x = 0.5f * t;                     // Height (now in x direction)
         float y = 0.3f + 0.5f * (1.0f - t * t); // Curved profile
         glVertex3f(x, y, -0.02f);               // Thin fin thickness
      }
      glVertex3f(0.5f, 0.3f, -0.02f); // Tip
      glEnd();

      // Other side of fin
      glBegin(GL_TRIANGLE_FAN);
      glVertex3f(0.0f, 0.3f, 0.0f);
      for (float t = 0.0f; t <= 1.0f; t += 0.1f)
      {
         float x = 0.5f * t;
         float y = 0.3f + 0.5f * (1.0f - t * t);
         glVertex3f(x, y, 0.02f);
      }
      glVertex3f(0.5f, 0.3f, 0.02f);
      glEnd();

      // Add fin edge detail
      glColor3f(0.6f, 0.6f, 0.6f); // Slightly darker for edges
      glBegin(GL_QUAD_STRIP);
      for (float t = 0.0f; t <= 1.0f; t += 0.1f)
      {
         float x = 0.5f * t;
         float y = 0.3f + 0.5f * (1.0f - t * t);
         glVertex3f(x, y, -0.02f);
         glVertex3f(x, y, 0.02f);
      }
      glVertex3f(0.5f, 0.3f, -0.02f);
      glVertex3f(0.5f, 0.3f, 0.02f);
      glEnd();

      // Add a reinforcement strip at the base
      glColor3f(0.65f, 0.65f, 0.65f);
      glBegin(GL_QUADS);
      glVertex3f(0.0f, 0.3f, -0.04f);
      glVertex3f(0.1f, 0.3f, -0.04f);
      glVertex3f(0.1f, 0.3f, 0.04f);
      glVertex3f(0.0f, 0.3f, 0.04f);
      glEnd();

      glPopMatrix();
   }

   // Add some details (stripes)
   glColor3f(0.8f, 0.2f, 0.2f);
   glPushMatrix();
   glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
   for (float z = 0.2f; z < 1.8f; z += 0.4f)
   {
      gluCylinder(quadric, 0.301f, 0.301f, 0.1f, 32, 32);
      glTranslatef(0.0f, 0.0f, 0.4f);
   }
   glPopMatrix();

   // Cleanup
   gluDeleteQuadric(quadric);

   // Optional: Add engine exhaust effect
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glColor4f(1.0f, 0.6f, 0.0f, 0.3f);
   glPushMatrix();
   glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
   glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
   gluCylinder(quadric, 0.2f, 0.1f, 0.6f, 32, 32);
   glPopMatrix();
   glDisable(GL_BLEND);
}

// Add these mouse callback functions
void mouse_button(int button, int state, int x, int y)
{
   if (button == GLUT_LEFT_BUTTON)
   {
      if (state == GLUT_DOWN)
      {
         mouse_left_pressed = true;
         mouse_last_x = x;
         mouse_last_y = y;
      }
      else
      {
         mouse_left_pressed = false;
      }
   }
}

void mouse_motion(int x, int y)
{
   if (mouse_left_pressed)
   {
      // Calculate change in mouse position
      int dx = x - mouse_last_x;
      dx = -dx;
      int dy = y - mouse_last_y;

      // Update camera angles (scale the rotation speed)
      camera_theta += dx * 0.5f;
      camera_phi += dy * 0.5f;

      // Clamp vertical angle to avoid gimbal lock
      if (camera_phi > 89.0f)
         camera_phi = 89.0f;
      if (camera_phi < -89.0f)
         camera_phi = -89.0f;

      // Store current mouse position for next frame
      mouse_last_x = x;
      mouse_last_y = y;

      // Request a redraw
      glutPostRedisplay();
   }
}

// Add mouse wheel zoom support
void mouse_wheel(int wheel, int direction, int x, int y)
{
   if (direction > 0)
   {
      // Zoom in
      camera_distance -= 0.5f;
      if (camera_distance < 2.0f)
         camera_distance = 2.0f;
   }
   else
   {
      // Zoom out
      camera_distance += 0.5f;
      if (camera_distance > 10.0f)
         camera_distance = 10.0f;
   }
   glutPostRedisplay();
}

// Display function to show current acceleration values
void display(void)
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glLoadIdentity();

   // Set up camera position
   float camera_x = camera_distance * cos(camera_phi * M_PI / 180.0f) * cos(camera_theta * M_PI / 180.0f);
   float camera_y = camera_distance * cos(camera_phi * M_PI / 180.0f) * sin(camera_theta * M_PI / 180.0f);
   float camera_z = camera_distance * sin(camera_phi * M_PI / 180.0f);

   gluLookAt(camera_x, camera_y, camera_z,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 1.0f);

   // Draw fixed coordinate axes first
   draw_coordinate_axes(3.0f);

   // Apply rotation only to the rocket
   glPushMatrix();
   float gl_matrix[16] = {
       current_rotation.data[0][0], current_rotation.data[1][0], current_rotation.data[2][0], 0.0f,
       current_rotation.data[0][1], current_rotation.data[1][1], current_rotation.data[2][1], 0.0f,
       current_rotation.data[0][2], current_rotation.data[1][2], current_rotation.data[2][2], 0.0f,
       0.0f, 0.0f, 0.0f, 1.0f};
   glMultMatrixf(gl_matrix);
   draw_rocket();
   glPopMatrix();

   // Draw UI text with acceleration values
   char info_text[128];
   snprintf(info_text, sizeof(info_text),
            "Camera: theta=%.1f° phi=%.1f° dist=%.1f",
            camera_theta, camera_phi, camera_distance);
   render_text_2d(10, window_height - 30, info_text);

   snprintf(info_text, sizeof(info_text),
            "Accel: X=%.2f Y=%.2f Z=%.2f",
            current_accel_x, current_accel_y, current_accel_z);
   render_text_2d(10, window_height - 60, info_text);

   // Inclination angle display
   double inclination = get_inclination_angle(current_rotation);
   snprintf(info_text, sizeof(info_text),
            "Inclination from Vertical: %.1f°",
            inclination * 180.0 / M_PI); // Convert radians to degrees
   render_text_2d(10, window_height - 90, info_text);

   render_text_2d(10, window_height - 120, "Left click + drag to rotate view");
   render_text_2d(10, window_height - 150, "Mouse wheel or +/- to zoom");
   render_text_2d(10, window_height - 180, "Keys 1-6 to change acceleration vector");

   glutSwapBuffers();
}

void reshape(int w, int h)
{
   window_width = w;
   window_height = h;

   glViewport(0, 0, w, h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(45.0f, (float)w / (float)h, 0.1f, 100.0f);
   glMatrixMode(GL_MODELVIEW);
}

void init_gl(void)
{
   glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
   glEnable(GL_DEPTH_TEST);
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_COLOR_MATERIAL);
   glShadeModel(GL_SMOOTH);

   // Set up lighting
   float light_position[] = {1.0f, 1.0f, 1.0f, 0.0f};
   float light_ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
   float light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};

   glLightfv(GL_LIGHT0, GL_POSITION, light_position);
   glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
   glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
}

// Add this function to smoothly update accelerometer values
void update_acceleration(int value)
{
   // Smoothly interpolate between current and target values
   const double interpolation_factor = 0.1; // Adjust for slower/faster transitions

   current_accel_x += (target_accel_x - current_accel_x) * interpolation_factor;
   current_accel_y += (target_accel_y - current_accel_y) * interpolation_factor;
   current_accel_z += (target_accel_z - current_accel_z) * interpolation_factor;

   // Update rotation matrix with new acceleration values
   current_rotation = create_rotation_matrix(
       current_accel_x,
       current_accel_y,
       current_accel_z);

   // Request a redraw
   glutPostRedisplay();

   // Schedule next update (16ms = ~60 FPS)
   glutTimerFunc(16, update_acceleration, 0);

   // Update animation time
   animation_time += 16;
}

// Add keyboard controls to set different orientations
void keyboard(unsigned char key, int x, int y)
{
   switch (key)
   {
   case '+':
   case '=':
      // Zoom in
      camera_distance -= 0.5f;
      if (camera_distance < 2.0f)
         camera_distance = 2.0f;
      break;
   case '-':
   case '_':
      // Zoom out
      camera_distance += 0.5f;
      if (camera_distance > 10.0f)
         camera_distance = 10.0f;
      break;
   case '1': // Flat
      target_accel_x = 0.0;
      target_accel_y = 0.0;
      target_accel_z = GRAVITY;
      break;
   case '2': // 45° tilt around X
      target_accel_x = 0.0;
      target_accel_y = -GRAVITY * 0.707; // sin(45°) * g
      target_accel_z = GRAVITY * 0.707;  // cos(45°) * g
      break;
   case '3': // Upside down
      target_accel_x = 0.0;
      target_accel_y = 0.0;
      target_accel_z = -GRAVITY;
      break;
   case '4':                          // 30° tilt around Y
      target_accel_x = GRAVITY * 0.5; // sin(30°) * g
      target_accel_y = 0.0;
      target_accel_z = GRAVITY * 0.866; // cos(30°) * g
      break;
   case '5':                            // 60° tilt around Y
      target_accel_x = GRAVITY * 0.866; // sin(60°) * g
      target_accel_y = 0.0;
      target_accel_z = GRAVITY * 0.5; // cos(60°) * g
      break;
   case '6':                             // 45° tilt around both X and Y
      target_accel_x = GRAVITY * 0.577;  // 1/√3 * g
      target_accel_y = -GRAVITY * 0.577; // 1/√3 * g
      target_accel_z = GRAVITY * 0.577;  // 1/√3 * g
      break;
   }
   glutPostRedisplay();
}

// Update visualize_rotation to start the animation
void visualize_rotation(Matrix3x3 rotation_matrix)
{
   current_rotation = rotation_matrix;

   int argc = 1;
   char *argv[1] = {(char *)"Rotation Visualization"};

   glutInit(&argc, argv);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutInitWindowSize(window_width, window_height);
   glutCreateWindow("Rotation Visualization");

   init_gl();

   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutMouseFunc(mouse_button);
   glutMotionFunc(mouse_motion);
   glutMouseWheelFunc(mouse_wheel);
   glutKeyboardFunc(keyboard);

   // Start the animation timer
   glutTimerFunc(16, update_acceleration, 0);

   glutMainLoop();
}
// END OPENGL CODE

// Helper function to print a Matrix3x3
void print_matrix(const char *label, Matrix3x3 m)
{
   printf("%s:\n", label);
   printf("⎡%6.3f %6.3f %6.3f⎤\n", m.data[0][0], m.data[0][1], m.data[0][2]);
   printf("⎢%6.3f %6.3f %6.3f⎥\n", m.data[1][0], m.data[1][1], m.data[1][2]);
   printf("⎣%6.3f %6.3f %6.3f⎦\n", m.data[2][0], m.data[2][1], m.data[2][2]);
}

int main(void)
{
   printf("\n=== Test Case 1: Device lying flat ===\n");
   printf("Acceleration vector: [%.3f, %.3f, %.3f]\n", 0.0, 0.0, GRAVITY);
   Matrix3x3 matrix_flat = create_rotation_matrix(0, 0, GRAVITY);
   print_matrix("Rotation matrix", matrix_flat);
   double inclination_flat = get_inclination_angle(matrix_flat);
   printf("Inclination: %.1f degrees\n\n", inclination_flat * 180.0 / M_PI);
   visualize_rotation(matrix_flat);

   printf("=== Test Case 2: Device tilted 45° around x-axis ===\n");
   printf("Acceleration vector: [%.3f, %.3f, %.3f]\n", 0.0, -6.93, 6.93);
   Matrix3x3 matrix_tilted = create_rotation_matrix(0, -6.93, 6.93);
   print_matrix("Rotation matrix", matrix_tilted);
   double inclination_tilted = get_inclination_angle(matrix_tilted);
   printf("Inclination: %.1f degrees\n\n", inclination_tilted * 180.0 / M_PI);
   // visualize_rotation(matrix_tilted);

   printf("=== Test Case 3: Device upside down ===\n");
   printf("Acceleration vector: [%.3f, %.3f, %.3f]\n", 0.0, 0.0, -GRAVITY);
   Matrix3x3 matrix_inverted = create_rotation_matrix(0, 0, -GRAVITY);
   print_matrix("Rotation matrix", matrix_inverted);
   double inclination_inverted = get_inclination_angle(matrix_inverted);
   printf("Inclination: %.1f degrees\n\n", inclination_inverted * 180.0 / M_PI);
   // visualize_rotation(matrix_inverted);

   printf("=== Test Case 4: Device tilted 30° around y-axis ===\n");
   printf("Acceleration vector: [%.3f, %.3f, %.3f]\n", 4.905, 0.0, 8.495);
   Matrix3x3 matrix_30deg = create_rotation_matrix(4.905, 0, 8.495);
   print_matrix("Rotation matrix", matrix_30deg);
   double inclination_30deg = get_inclination_angle(matrix_30deg);
   printf("Inclination: %.1f degrees\n\n", inclination_30deg * 180.0 / M_PI);
   // visualize_rotation(matrix_30deg);

   printf("=== Test Case 5: Device tilted 60° around y-axis ===\n");
   printf("Acceleration vector: [%.3f, %.3f, %.3f]\n", 8.495, 0.0, 4.905);
   Matrix3x3 matrix_60deg = create_rotation_matrix(8.495, 0, 4.905);
   print_matrix("Rotation matrix", matrix_60deg);
   double inclination_60deg = get_inclination_angle(matrix_60deg);
   printf("Inclination: %.1f degrees\n\n", inclination_60deg * 180.0 / M_PI);
   // visualize_rotation(matrix_60deg);

   printf("=== Test Case 6: Device tilted diagonally (45° around both x and y axes) ===\n");
   printf("Acceleration vector: [%.3f, %.3f, %.3f]\n", 5.67, -5.67, 4.905);
   Matrix3x3 matrix_diagonal = create_rotation_matrix(5.67, -5.67, 4.905);
   print_matrix("Rotation matrix", matrix_diagonal);
   double inclination_diagonal = get_inclination_angle(matrix_diagonal);
   printf("Inclination: %.1f degrees\n\n", inclination_diagonal * 180.0 / M_PI);
   // visualize_rotation(matrix_diagonal);

   return 0;
}