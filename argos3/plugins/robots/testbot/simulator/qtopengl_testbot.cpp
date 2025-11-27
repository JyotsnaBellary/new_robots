/**
 * @file <testbot/simulator/qtopengl_testbot.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "qtopengl_testbot.h"
#include "testbot_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_measures.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* All measures are in meters */

   static const Real WHEEL_DIAMETER              = TURTLEBOT4_WHEEL_RADIUS * 2.0f;
   static const Real WHEEL_RADIUS                = TURTLEBOT4_WHEEL_RADIUS;
   static const Real WHEEL_WIDTH                 = 0.02f; // Need this value
   static const Real HALF_WHEEL_WIDTH            = WHEEL_WIDTH * 0.5f;
   static const Real INTERWHEEL_DISTANCE         = 0.053f;
   static const Real HALF_INTERWHEEL_DISTANCE    = TURTLEBOT4_HALF_WHEEL_DISTANCE;

   // static const Real CHASSIS_ELEVATION           = 0.005f;                            // to be checked!
   // static const Real HALF_CHASSIS_LENGTH         = 0.0275f;                             // to be checked!
   static const Real HALF_CHASSIS_WIDTH          = TURTLEBOT4_HALF_WHEEL_DISTANCE - HALF_WHEEL_WIDTH;

   static const Real BODY_RADIUS                 = TURTLEBOT4_BASE_RADIUS;
   static const Real BODY_ELEVATION              = TURTLEBOT4_BASE_ELEVATION; // to be checked!
   static const Real BODY_HEIGHT                 = TURTLEBOT4_BASE_HEIGHT;                              // to be checked!
   static const Real LOWER_BODY_HEIGHT          = TURTLEBOT4_LOWER_BODY_HEIGHT;

   static const Real LED_ELEVATION               = BODY_ELEVATION + BODY_HEIGHT;
   static const Real LED_HEIGHT                  = 0.01;                               // to be checked!
   // static const Real LED_UPPER_RING_INNER_RADIUS = 0.8 * BODY_RADIUS;

   /****************************************/
   /****************************************/

   CQTOpenGLTestBot::CQTOpenGLTestBot() :
      m_unVertices(40)
      // m_fLEDAngleSlice(360.0f / 8.0f) 
      {
      /* Reserve the needed display lists */
      m_unLists = glGenLists(4);

      /* Assign indices for better referencing (later) */
      m_unWheelList   = m_unLists;
      // m_unChassisList = m_unLists + 1;
      m_unBodyList    = m_unLists + 1;
      // m_unLEDList     = m_unLists + 2;
      m_unUpperBodyList = m_unLists + 2;
      m_unColumnList    = m_unLists + 3;

      /* Create the wheel display list */
      glNewList(m_unWheelList, GL_COMPILE);
      RenderWheel();
      glEndList();

      /* Create the body display list */
      glNewList(m_unBodyList, GL_COMPILE);
      RenderBody();
      glEndList();

      /* Create the upper body display list */
      glNewList(m_unUpperBodyList, GL_COMPILE);
      RenderUpperBody();
      glEndList();

      /* Create the column display list */
      glNewList(m_unColumnList, GL_COMPILE);
      RenderColumn();
      glEndList();

      /* Create the chassis display list */
      // glNewList(m_unChassisList, GL_COMPILE);
      // RenderChassis();
      // glEndList();

      /* Create the LED display list */
      // glNewList(m_unLEDList, GL_COMPILE);
      // RenderLED();
      // glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLTestBot::~CQTOpenGLTestBot() {
      glDeleteLists(m_unLists, 4);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLTestBot::Draw(CTestBotEntity& c_entity) {
      /* Place the chassis */
      // glCallList(m_unChassisList);
      /* Place the body */
      glCallList(m_unBodyList);
      /* Place the wheels */
      glPushMatrix();
      glTranslated(0.0f, HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
      glPushMatrix();
      glTranslated(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
      /* Place the LEDs */
      // glPushMatrix();

      // from testbot_entity.h

      // CLEDEquippedEntity& cLEDEquippedEntity = c_entity.GetLEDEquippedEntity();
      // for(UInt32 i = 0; i < 8; i++) {
      //    const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
      //    // glRotated(-m_fLEDAngleSlice, 0.0f, 0.0f, 1.0f);
      //    // SetLEDMaterial(cColor.GetRed(),
      //    //                cColor.GetGreen(),
      //    //                cColor.GetBlue());
      //    // glCallList(m_unLEDList);
      // }
   glCallList(m_unUpperBodyList);

      /* Columns (3 pillars) */
   CRadians cStep = CRadians::TWO_PI / TURTLEBOT4_NUM_COLUMNS;
   Real angleStep = cStep.GetValue();

   Real radius    = BODY_RADIUS * 0.9f; // slightly inside edge

   for(UInt32 i = 0; i < TURTLEBOT4_NUM_COLUMNS; ++i) {
    CRadians cAngle = cStep * i;

    glPushMatrix();
    glTranslated(radius * Cos(cAngle),
                 radius * Sin(cAngle),
                 0.0f);
    glCallList(m_unColumnList);
    glPopMatrix();
}
      // glPopMatrix();
      /* Upper body plate */
   }

   /****************************************/
   /****************************************/

   // void CQTOpenGLTestBot::SetGreenPlasticMaterial() {
   //    const GLfloat pfColor[]     = {   0.0f, 1.0f, 0.0f, 1.0f };
   //    const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
   //    const GLfloat pfShininess[] = { 100.0f                   };
   //    const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   // }

   /****************************************/
   /****************************************/

   // void CQTOpenGLTestBot::SetRedPlasticMaterial() {
   //    const GLfloat pfColor[]     = {   1.0f, 0.0f, 0.0f, 1.0f };
   //    const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
   //    const GLfloat pfShininess[] = { 100.0f                   };
   //    const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   // }

   /****************************************/
   /****************************************/

   void CQTOpenGLTestBot::SetCircuitBoardMaterial() {
      const GLfloat pfColor[]     = { 0.0f, 0.0f, 1.0f, 1.0f };
      const GLfloat pfSpecular[]  = { 0.5f, 0.5f, 1.0f, 1.0f };
      const GLfloat pfShininess[] = { 10.0f                  };
      const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   // void CQTOpenGLTestBot::SetLEDMaterial(GLfloat f_red,
   //                                     GLfloat f_green,
   //                                     GLfloat f_blue) {
   //    const GLfloat pfColor[]     = { f_red, f_green, f_blue, 1.0f };
   //    const GLfloat pfSpecular[]  = {  0.0f,    0.0f,   0.0f, 1.0f };
   //    const GLfloat pfShininess[] = {  0.0f                        };
   //    const GLfloat pfEmission[]  = { f_red, f_green, f_blue, 1.0f };
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
   //    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   // }

   /****************************************/
   /****************************************/

   void CQTOpenGLTestBot::RenderWheel() {
      /* Set material */
      // SetRedPlasticMaterial();
      /* Right side */
      CVector2 cVertex(WHEEL_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector3 cNormal(-1.0f, -1.0f, 0.0f);
      cNormal.Normalize();
      glBegin(GL_POLYGON);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3d(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Left side */
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cNormal.Set(-1.0f, 1.0f, 0.0f);
      cNormal.Normalize();
      cAngle = -cAngle;
      glBegin(GL_POLYGON);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3d(cVertex.GetX(), HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Tire */
      cNormal.Set(1.0f, 0.0f, 0.0f);
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cAngle = -cAngle;
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3d(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         glVertex3d(cVertex.GetX(),  HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
   }

   void CQTOpenGLTestBot::RenderUpperBody() {
    Real z = BODY_ELEVATION + LOWER_BODY_HEIGHT + TURTLEBOT4_COLUMN_HEIGHT;

    CVector2 cVertex(UPPER_BODY_RADIUS, 0.0f);
    CRadians cAngle(CRadians::TWO_PI / m_unVertices);

    glBegin(GL_POLYGON);
    glNormal3d(0.0f, 0.0f, 1.0f);
    for (GLuint i = 0; i <= m_unVertices; i++) {
        glVertex3d(cVertex.GetX(), cVertex.GetY(), z);
        cVertex.Rotate(cAngle);
    }
    glEnd();
}




   void CQTOpenGLTestBot::RenderBody() {
   /* Bottom disk */
   CVector2 cVertex(BODY_RADIUS, 0.0f);
   CRadians cAngle(-CRadians::TWO_PI / m_unVertices);

   glBegin(GL_POLYGON);
   glNormal3d(0.0f, 0.0f, -1.0f);
   for(GLuint i = 0; i <= m_unVertices; i++) {
      glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
      cVertex.Rotate(cAngle);
   }
   glEnd();

   /* Side cylinder */
   cAngle = -cAngle;
   CVector2 cNormal(1.0f, 0.0f);
   cVertex.Set(BODY_RADIUS, 0.0f);

   glBegin(GL_QUAD_STRIP);
   for(GLuint i = 0; i <= m_unVertices; i++) {
      glNormal3d(cNormal.GetX(), cNormal.GetY(), 0.0f);
      glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + LOWER_BODY_HEIGHT);
      glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
      cVertex.Rotate(cAngle);
      cNormal.Rotate(cAngle);
   }
   glEnd();

   /* NEW: Top disk */
   cVertex.Set(BODY_RADIUS, 0.0f);
   glBegin(GL_POLYGON);
   glNormal3d(0.0f, 0.0f, 1.0f);
   for(GLuint i = 0; i <= m_unVertices; i++) {
      glVertex3d(cVertex.GetX(), cVertex.GetY(),
                 BODY_ELEVATION + LOWER_BODY_HEIGHT);
      cVertex.Rotate(cAngle);
   }
   glEnd();
}


void CQTOpenGLTestBot::RenderColumn() {
   Real baseZ = BODY_ELEVATION + LOWER_BODY_HEIGHT;
   Real topZ  = baseZ + TURTLEBOT4_COLUMN_HEIGHT;

   CVector2 cNormal(1.0f, 0.0f);
   CVector2 cVertex(TURTLEBOT4_COLUMN_RADIUS, 0.0f);
   CRadians cAngle(CRadians::TWO_PI / m_unVertices);

   /* Side tube */
   glBegin(GL_QUAD_STRIP);
   for(UInt32 i = 0; i <= m_unVertices; ++i) {
      glNormal3d(cNormal.GetX(), cNormal.GetY(), 0.0f);
      glVertex3d(cVertex.GetX(), cVertex.GetY(), baseZ);
      glVertex3d(cVertex.GetX(), cVertex.GetY(), topZ);
      cVertex.Rotate(cAngle);
      cNormal.Rotate(cAngle);
   }
   glEnd();
}


   /****************************************/
   /****************************************/

   // void CQTOpenGLTestBot::RenderChassis() {
   //    /* Set material */
   //    SetGreenPlasticMaterial();
   //    /* This part covers the bottom face (parallel to XY) */
   //    glBegin(GL_QUADS);
   //    /* Bottom face */
   //    glNormal3d(0.0f, 0.0f, -1.0f);
   //    glVertex3d( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    glVertex3d( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    glVertex3d(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    glVertex3d(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    glEnd();
   //    /* This part covers the faces (South, East, North, West) */
   //    glBegin(GL_QUAD_STRIP);
   //    /* Starting side */
   //    glNormal3d(-1.0f, 0.0f, 0.0f);
   //    glVertex3d(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
   //    glVertex3d(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    /* South face */
   //    glVertex3d( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
   //    glVertex3d( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    /* East face */
   //    glNormal3d(0.0f, -1.0f, 0.0f);
   //    glVertex3d( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
   //    glVertex3d( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    /* North face */
   //    glNormal3d(1.0f, 0.0f, 0.0f);
   //    glVertex3d(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
   //    glVertex3d(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    /* West face */
   //    glNormal3d(0.0f, 1.0f, 0.0f);
   //    glVertex3d(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
   //    glVertex3d(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
   //    glEnd();
   // }

   /****************************************/
   /****************************************/

   // void CQTOpenGLTestBot::RenderBody() {
   //    /* Set material */
   //    // SetGreenPlasticMaterial();
   //    CVector2 cVertex(BODY_RADIUS, 0.0f);
   //    CRadians cAngle(-CRadians::TWO_PI / m_unVertices);
   //    /* Bottom part */
   //    glBegin(GL_POLYGON);
   //    glNormal3d(0.0f, 0.0f, -1.0f);
   //    for(GLuint i = 0; i <= m_unVertices; i++) {
   //       glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
   //       cVertex.Rotate(cAngle);
   //    }
   //    glEnd();
   //    /* Side surface */
   //    cAngle = -cAngle;
   //    CVector2 cNormal(1.0f, 0.0f);
   //    cVertex.Set(BODY_RADIUS, 0.0f);
   //    glBegin(GL_QUAD_STRIP);
   //    for(GLuint i = 0; i <= m_unVertices; i++) {
   //       glNormal3d(cNormal.GetX(), cNormal.GetY(), 0.0f);
   //       glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + LOWER_BODY_HEIGHT);
   //       glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
   //       cVertex.Rotate(cAngle);
   //       cNormal.Rotate(cAngle);
   //    }
   //    glEnd();
   //    /* Top part */
   //    // glBegin(GL_POLYGON);
   //    // cVertex.Set(LED_UPPER_RING_INNER_RADIUS, 0.0f);
   //    // glNormal3d(0.0f, 0.0f, 1.0f);
   //    // for(GLuint i = 0; i <= m_unVertices; i++) {
   //    //    glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
   //    //    cVertex.Rotate(cAngle);
   //    // }
   //    // glEnd();
   //    /* Triangle to set the direction */
   //    // SetLEDMaterial(1.0f, 1.0f, 0.0f);
   //    // glBegin(GL_TRIANGLES);
   //    // glVertex3d( BODY_RADIUS * 0.7,               0.0f, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
   //    // glVertex3d(-BODY_RADIUS * 0.7,  BODY_RADIUS * 0.3, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
   //    // glVertex3d(-BODY_RADIUS * 0.7, -BODY_RADIUS * 0.3, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
   //    // glEnd();
   // }

   /****************************************/
   /****************************************/

   // void CQTOpenGLTestBot::RenderLED() {
   //    /* Side surface */
   //    CVector2 cVertex(BODY_RADIUS, 0.0f);
   //    CRadians cAngle(CRadians::TWO_PI / m_unVertices);
   //    CVector2 cNormal(1.0f, 0.0f);
   //    glBegin(GL_QUAD_STRIP);
   //    for(GLuint i = 0; i <= m_unVertices / 8; i++) {
   //       glNormal3d(cNormal.GetX(), cNormal.GetY(), 0.0f);
   //       glVertex3d(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION + LED_HEIGHT);
   //       glVertex3d(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION);
   //       cVertex.Rotate(cAngle);
   //       cNormal.Rotate(cAngle);
   //    }
   //    glEnd();
   //    /* Top surface  */
   //    cVertex.Set(BODY_RADIUS, 0.0f);
   //    CVector2 cVertex2(LED_UPPER_RING_INNER_RADIUS, 0.0f);
   //    glBegin(GL_QUAD_STRIP);
   //    glNormal3d(0.0f, 0.0f, 1.0f);      
   //    for(GLuint i = 0; i <= m_unVertices / 8; i++) {         
   //       glVertex3d(cVertex2.GetX(), cVertex2.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
   //       glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
   //       cVertex.Rotate(cAngle);
   //       cVertex2.Rotate(cAngle);
   //    }
   //    glEnd();
   // }

   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawTestBotNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CTestBotEntity& c_entity) {
         static CQTOpenGLTestBot m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawTestBotSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CTestBotEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawTestBotNormal, CTestBotEntity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawTestBotSelected, CTestBotEntity);

   /****************************************/
   /****************************************/

}
