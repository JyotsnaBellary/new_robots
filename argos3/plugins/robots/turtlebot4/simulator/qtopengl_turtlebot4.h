/**
 * @file <turtlebot4/simulator/qtopengl_turtlebot4.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef QTOPENGL_TURTLEBOT4_H
#define QTOPENGL_TURTLEBOT4_H

namespace argos {
   class CQTOpenGLTurtlebot4;
   class CTurtlebot4Entity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLTurtlebot4 {

   public:

      CQTOpenGLTurtlebot4();

      virtual ~CQTOpenGLTurtlebot4();

      virtual void Draw(CTurtlebot4Entity& c_entity);

   protected:

      /** Renders a wheel */
      void RenderWheel();

      /** Renders the body */
      void RenderBody();

      /** Renders the upperbody */
      void RenderUpperBody();

      /** Renders the columns */
      void RenderColumn();

      /** Set Base Material */ 
      void SetBaseMaterial();

      /** Set deck Material */ 
      void SetDeckMaterial();

      /** Set Column Material */ 
      void SetColumnMaterial();

      /** Set Wheel Material */
      void SetWheelMaterial(); 

      /** Renders the camera */
      void RenderCamera();

      void SetWhitePlasticMaterial();
   private:

      /** Start of the display list index */
      GLuint m_unLists;

      /** Turtlebot4 wheel */
      GLuint m_unWheelList;

      /** Body display list */
      GLuint m_unBodyList;

      /** Upper body list */
      GLuint m_unUpperBodyList;

      /** Column list */
      GLuint m_unColumnList;

      /** Number of vertices to display the round parts */
      GLuint m_unVertices;

      /** Foot-bot camera module */
      GLuint m_unCameraList;
   };

}

#endif
