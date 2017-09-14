#include "ui_engine.h"

#include <string.h>

#include <GL/glut.h>
#include <GL/freeglut.h>

using namespace semantic_mapping::demo;

UiEngine* UiEngine::instance_;

static void GlutBitmapString(void* font, const char* str) {
  size_t len = strlen(str);
  for(size_t i=0; i<len; i++)
    glutBitmapCharacter(font, str[i]);
} 

void UiEngine::GlutDisplayFunction() {
  UiEngine* ui_engine = UiEngine::Instance();
  ui_engine->main_engine_->GetImage(ui_engine->out_image_[0], 
                                    ui_engine->out_image_type_[0],
                                    &ui_engine->free_view_pose_,
                                    &ui_engine->free_view_intrinsics_);
  for(int w=1; w<NUM_WIN; w++)
    ui_engine->main_engine_->GetImage(ui_engine->out_image_[w], 
                                      ui_engine->out_image_type_[w]);
  glClear(GL_COLOR_BUFFER_BIT);
  glColor3f(1.0f, 1.0f, 1.0f);
  glEnable(GL_TEXTURE_2D);
  Uchar4Image** show_images = ui_engine->out_image_;
  Vector4f* win_reg = ui_engine->win_reg_;
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  
  glLoadIdentity();
  glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  glEnable(GL_TEXTURE_2D);
  for(int w=0; w<NUM_WIN; w++) {
    if(ui_engine->out_image_type_[w] == MainEngine::IMAGE_UNKNOWN)
      continue;
    glBindTexture(GL_TEXTURE_2D, ui_engine->texture_id_[w]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, show_images[w]->num_dims_.x, 
                 show_images[w]->num_dims_.y, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 show_images[w]->GetData(MEMEORY_DEVICE_CPU));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glBegin(GL_QUADS);

    glTexCoord2f(0, 1);
    glVertex2f(win_reg[w][0], win_reg[w][1]);
    glTexCoord2f(1, 1);
    glVertex2f(win_reg[w][2], win_reg[w][1]);
    glTexCoord2f(1, 0);
    glVertex2f(win_reg[w][2], win_reg[w][3]);
    glTexCoord2f(0, 0);
    glVertex2f(win_reg[w][0], win_reg[w][3]);

    glEnd();
  } 
  glDisable(GL_TEXTURE_2D);
  
  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glColor3f(1.0f, 0.0f, 0.0f);
  glRasterPos2f(0.85f, -0.962f);
  
  char str[200];
  sprintf(str, "%04.2lf", ui_engine->processed_time_);
  GlutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*)str);
  glRasterPos2f(-0.95f, -0.95f);

  GlutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*)str);
  glutSwapBuffers();
  ui_engine->need_refresh_ = false; 
}

void UiEngine::GlutIdleFunction() {
  UiEngine* ui_engine = UiEngine::Instance();
  switch(ui_engine->main_loop_action_) {
    case PROCESS_FRAME:
      ui_engine->ProcessFrame();
      ui_engine->processed_frame_num_++;
      ui_engine->main_loop_action_ = PROCESS_PAUSED;
      ui_engine->need_refresh_ = true;
      break;
    case PROCESS_VIDEO:
      ui_engine->ProcessFrame();
      ui_engine->processed_frame_num_++;
      ui_engine->need_refresh_ = true;
      break; 
    case EXIT:
      glutLeaveMainLoop();
      break;
    case PROCESS_PAUSED:
    case default:
      break;
  }
  if(ui_engine->need_refresh_)
    glutPostRedisplay();
}

void UiEngine::GlutKeyUpFunction(unsigned char key, int x, int y) {
  UiEngine* ui_engine = UiEngine::Instance();
  switch(key) {
    case 'n':
      printf("processing one frame ...\n");
      ui_engine->main_loop_action_ = ui_engine::PROCESS_FRAME;
      break;
    case 'a':
      printf("processing input source ...\n");
      ui_engine->main_loop_action_ = ui_engine::PROCESS_VIDEO;
      break;
    case 's':
      if(ui_engine->is_recording) {
        printf("stopped recording disk ...\n");
        ui_engine->is_recording_ = false;
      } else {
        printf("started recording disk ...\n");
        ui_engine->current_frame_num_ = 0;
        ui_engine->is_recording_ = true;
      }
      break;
    case 'e':
    case 27:
      printf("exiting ...\n");
      ui_engine->main_loop_action_ = ui_engine::EXIT;
      break;
    case 'f':
      if(ui_engine->free_view_active_) {
        ui_engine->out_image_type_[0] = MainEngine::IMAGE_SCENERAYCAST;
        ui_engine->out_image_type_[1] = MainEngine::IMAGE_ORIGIN_DEPTH;
        ui_engine->free_view_active_ = false;
      } else {
        ui_engine->out_image_type_[0] = MainEngine::IMAGE_FREE_CAMERA_SHADED;
        ui_engine->out_image_type_[1] = MainEngine::IMAGE_SCENERAYCAST;
        ui_engine->free_view_pose_.SetFrom(
                   ui_engine->main_engine_->GetTrackingState()->pose_depth_);
        if(ui_engine->main_engine_->GetView() != NULL) {
          ui_engine->free_view_intrinsics_ = 
            ui_engine->main_engine_->GetView()->rgbd_calib_->intrinsics_depth_;
          ui_engine->out_image_[0]->ChangeDimension(
            ui_engine->main_engine_->GetView()->depth_->num_dims_);
        }
        ui_engine->free_view_active_ = true;
      }
      ui_engine->need_refresh_ = true;
      break;
    case 'c':
      ui_engine->current_color_mode_++;
      if((unsigned)ui_engine->current_color_mode_>=ui_engine->color_modes_.size())
        ui_engine->current_color_mode_ = 0;
      ui_engine->need_refresh_ = true;
      break;
    case 't':
      ui_engine->intergration_active_ = !ui_engine->intergration_active_;
      if(ui_engine->intergration_active_)
        ui_engine->main_engine_->TurnOnIntegration();
      else 
        ui_engine->main_engine_->TurnOffIntegration();
      break;
    case 'w':
      printf("saving mesh to disk ...\n");
      ui_engine->SaveSceneToMesh("mesh.stl");
      printf("done\n");
      break;
    default:
      break; 
  }
  if(ui_engine->free_view_active_)
    ui_engine->out_image_type_[0] = 
       ui_engine->color_modes_[ui_engine->current_color_mode_].type;
}

void UiEngine::GlutMouseButtonFunction(int button, int state, int x, int y) {
  UiEngine* ui_engine = UiEngine::Instance();
  if(state == GLUT_DOWN) {
    switch(button) {
      case GLUT_LEFT_BUTTON:
        ui_engine->mouse_state_ = 1;
        break;
      case GLUT_MIDDLE_BUTTON:
        ui_engine->mouse_state_ = 3;
        break;
      case GLUT_RIGHT_BUTTON:
        ui_engine->mouse_state_ = 2;
        break;
      default:
        break;
    }
    ui_engine->mouse_last_click_.x = x;
    ui_engine->mouse_last_click_.y = y;
  } else if(state == GLUT_UP) {
    ui_engine->mouse_state_ = 0;
  }
}

static inline Matrix3f CreateRotation(const Vector3f & _axis, float angle) {                                                                               
  Vector3f axis = normalize(_axis);                                           
  float si = sinf(angle);                                                     
  float co = cosf(angle);                                                     
  Matrix3f ret;                                                               
  ret.SetIdentity();                                                          
  ret *= co;                                                                  

  for(int r = 0; r < 3; ++r) 
    for(int c = 0; c < 3; ++c) 
      ret.at(c, r) += (1.0f - co) * axis[c] * axis[r];

  Matrix3f skewmat;                                                           
  skewmat.SetZeros();                                                         
  skewmat.at(1, 0) = -axis.z;                                                 
  skewmat.at(0, 1) = axis.z;                                                  
  skewmat.at(2, 0) = axis.y;                                                  
  skewmat.at(0, 2) = -axis.y;                                                 
  skewmat.at(2, 1) = axis.x;                                                  
  skewmat.at(1, 2) = -axis.x;                                                 
  skewmat *= si;                                                              
  ret += skewmat;                                                             

  return ret;                                                                 
}                

void UiEngine::GlutMouseMoveFunction(int x, int y) {
  UiEngine* ui_engine = UiEngine::Instance();
  if(!ui_engine->free_view_active_)
    return;
  Vector2i movement;
  movement.x = x-ui_engine->mouse_last_click_.x;
  movement.y = u-ui_engine->mouse_last_click_.y;
  ui_engine->mouse_last_click_.x = x;
  ui_engine->mouse_last_click_.y = y;
  if((movement.x==0)&&(movement.y==0))
    return;
  static const float scale_rotation = 0.005f;
  static const float scale_translation = 0.0025f;
  
  switch(ui_engine->mouse_state_) {
    case 1:
      Vector3f axis((float)-movement.y, (float)-movement.x, 0.0f);
      float angle = scale_rotation*sqrt((float)(movement.x*movement.x
                                               +movement.y*movement.y));
      Matrix3f rot = CreateRotation(axis, angle);
      ui_engine->free_view_pose_.SetRotationTranslation(
                 rot*ui_engine->free_view_pose_.GetRoation(), 
                 rot*ui_engine->free_view_pose_.GetTranslation());
      ui_engine->free_view_pose_->Coerce();
      ui_engine->need_refresh_ = true;
      break;
    case 2:
      ui_engine->free_view_pose_.SetTranslation(
        ui_engine->free_view_pose_.GetTranslation()+
        scale_translation*Vector3f((float)movement.x, (float)movement.y, 0.0f));
      ui_engine->need_refresh_ = true;
      break;
    case 3:
      ui_engine->free_view_pose_.SetTranslation(
        ui_engine->free_view_pose_.GetTranslation()+
        scale_translation*Vector3f(0.0f, 0.0f, (float)movement.y));
      ui_engine->need_refresh_ = true;
      break;
    default:
      break;
  }
}

