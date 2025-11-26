#define GL_SILENCE_DEPRECATION
#include <GLUT/glut.h>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <algorithm>
#include <iostream>
#include <GL/glui.h>
using namespace glm;
using namespace std;
int main_window;
float camX=-19.38f,camY=7.03f,camZ=18.40f;
float camyaw=-40.90f;
float campitch=-12.0f;
float sens=0.3f;
float speed=0.1f;
float lastMouseX=0,lastMouseY=0;
bool leftDown=false;
bool rightDown=false;
float g_vision_radius = 1.0f;
float g_safe_bubble = 0.5f;
float g_align_weight = 1.2f;
float g_cohesion_weight = 0.8f;
float g_separation_weight = 2.5f;
class Boid{
    private:
        vec3 position;
        vec3 velocity;
        vec3 acceleration; 
    public:
        float vision_radius;
        float vision_angle;
        float safe_bubble;
        float sep_weight;
        float align_weight;
        float cohesion_weight;
        float obs_avoid_weight;
        float max_speed;
        float max_acc;
        float max_force;
        
    public:
        Boid(vec3 pos) {
            position = pos;
            velocity = vec3((rand()%100)/80.0f-0.5f, 
                            (rand()%100)/80.0f-0.5f,
                            (rand()%100)/80.0f-0.5f);
            if (length(velocity)<1e-6f){
                velocity = vec3(0.1f,0,0);
            }
            acceleration=vec3(0);
            max_speed=0.2f;
            max_acc=0.06f;
            max_force=0.1f;
            vision_radius=g_vision_radius;
            vision_angle=radians(270.0f);
            safe_bubble= g_safe_bubble;
            sep_weight=g_separation_weight;
            align_weight=g_align_weight;
            cohesion_weight=g_cohesion_weight;
            obs_avoid_weight=2.3f;
        }
        vec3 ret_position(){
            return this->position;
        }
        vec3 ret_velocity(){
            return this->velocity;
        }
        vec3 ret_acceleration(){
            return this->acceleration;
        }
        void apply_force(vec3 f) {
            acceleration+=f;
        }
        void update() {
            vision_radius=g_vision_radius;
            safe_bubble= g_safe_bubble;
            sep_weight=g_separation_weight;
            align_weight=g_align_weight;
            cohesion_weight=g_cohesion_weight;
            velocity+=acceleration;
            if (length(velocity)>max_speed)
                velocity=normalize(velocity)*max_speed;
            position+=velocity;
            if (position.x > 5) {
                position.x = 5;
                velocity.x = -velocity.x;
            }
            if (position.x < -5) {
                position.x = -5;
                velocity.x = -velocity.x;
            }

            if (position.y > 5) {
                position.y = 5;
                velocity.y = -velocity.y;
            }
            if (position.y < -5) {
                position.y = -5;
                velocity.y = -velocity.y;
            }

            if (position.z > 5) {
                position.z = 5;
                velocity.z = -velocity.z;
            }
            if (position.z < -5) {
                position.z = -5;
                velocity.z = -velocity.z;
            }
            acceleration=vec3(0); 
        }

};
class Obstacle{
    public:
        virtual bool isviewable(
            Boid *specific_boid
        )=0;
        virtual vec3 avoid_force(Boid*specific_boid)=0;
        virtual void draw()=0;
        virtual ~Obstacle(){}
};
class circular_obs: public Obstacle{
    vec3 center_point;
    float radius;
    public:circular_obs(const vec3& pos, float r)
    : center_point(pos), radius(r) {}
    bool isviewable(Boid *specific_boid) override{
        vec3 dir=specific_boid->ret_velocity();
        dir=normalize(dir);
        vec3 pos=specific_boid->ret_position();
        //the condition for viewablity is that dist<obs bubble.
        float dist=length(pos-center_point);
        if(dist>radius+specific_boid->vision_radius*4.5f) return false;
        vec3 ptr_vector=center_point-pos;
        float angle=acos(std::clamp(dot(dir,normalize(ptr_vector)),-1.0f,1.0f));
        if(angle<specific_boid->vision_angle*0.5f){
            return true;
        }
        return false;
    }
    vec3 avoid_force(Boid*specific_boid) override{
        vec3 away=specific_boid->ret_position()-center_point;
        float dist=length(away);
        float max_dist=specific_boid->vision_radius+radius;
        float penetration=pow((max_dist-dist)/max_dist,2.0f);
        penetration=std::clamp(penetration,0.0f,1.0f);
        return normalize(away)*penetration;
    }
    void draw()override{
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINE_LOOP);
        for(int i=0;i<60;i++){
            float theta=2.0f*M_PI*float(i)/60.0f;
            float x=center_point.x+radius*cos(theta);
            float y=center_point.y+radius*sin(theta);
            glVertex2f(x,y);
        }
        glEnd();
    }
};
class square_obs : public Obstacle{
    vec3 center_point;
    float side;
    public:square_obs(const vec3&pos,float r):center_point(pos),side(r){}
    bool isviewable(Boid *specific_boid)override{
        vec3 pos = specific_boid->ret_position();
        vec3 vel = normalize(specific_boid->ret_velocity());
        if (length(vel)<1e-5)vel = vec3(1,0,0);
        vec3 half(side/2.0f);
        vec3 diff=pos-center_point;
        vec3 closest =center_point+clamp(diff,-half,half);
        float dist =length(pos-closest);   
        if (dist > specific_boid->vision_radius) return false;
        vec3 dir_to_obs=normalize(closest-pos);
        float angle=acos(std::clamp(dot(vel, dir_to_obs),-1.0f,1.0f));
        return angle<specific_boid->vision_angle*0.5f;
    }
    vec3 avoid_force(Boid *specific_boid)override{
        vec3 pos = specific_boid->ret_position();
        vec3 half(side/2.0f);
        vec3 diff =pos-center_point;
        vec3 closest=center_point+clamp(diff,-half,half);
        float dist = length(pos-closest);
        float strength=1.0f-(dist/specific_boid->vision_radius);
        strength=std::clamp(strength, 0.0f, 1.0f);
        vec3 away =normalize(pos-closest);
        return away*strength;
    }
    void draw()override{
        glColor3f(0.0f,0.0f,1.0f);  
        float h =side/2.0f;
        glBegin(GL_LINE_LOOP);
        glVertex2f(center_point.x-h,center_point.y-h);
        glVertex2f(center_point.x+h,center_point.y-h);
        glVertex2f(center_point.x+h,center_point.y+h);
        glVertex2f(center_point.x-h,center_point.y+h);
        glEnd();
    }
};
class sphere_obs:public Obstacle{
    vec3 center_point;
    float radius;

    public:
        sphere_obs(const vec3& pos,float r)
        :center_point(pos),radius(r){}
    bool isviewable(Boid *specific_boid) override{
        vec3 dir=specific_boid->ret_velocity();
        dir=normalize(dir);
        vec3 pos=specific_boid->ret_position();
        //the condition for viewablity is that dist<obs bubble.
        float dist=length(pos-center_point);
        if(dist>radius+specific_boid->vision_radius*4.5f) return false;
        vec3 ptr_vector=center_point-pos;
        float angle=acos(std::clamp(dot(dir,normalize(ptr_vector)),-1.0f,1.0f));
        if(angle<specific_boid->vision_angle*0.5f){
            return true;
        }
        return false;
    }
    vec3 avoid_force(Boid *specific_boid) override {
        vec3 pos = specific_boid->ret_position();
        vec3 away = pos-center_point;
        float dist=length(away);
        float max_dist = specific_boid->vision_radius+radius;
        float penetration = pow((max_dist-dist)/max_dist,2.0f);
        penetration = std::clamp(penetration,0.0f,1.0f);
        return normalize(away)*penetration;
    }
    void draw() override {
        glDisable(GL_BLEND); 
        glColor3f(0.0f,0.5f,1.0f);
        glPushMatrix();
        glTranslatef(center_point.x,center_point.y,center_point.z);
        glutSolidSphere(radius,24,24);
        glPopMatrix();
        glEnable(GL_BLEND);
    }
};
vector<Boid*>all_boids;
vector<Obstacle*>all_obs;
float ret_radians(float x){
    return x*(M_PI/180.0f);
}
void getCameraDir(float &dirX,float &dirY, float &dirZ){
    dirX=cosf(ret_radians(camyaw))*cosf(ret_radians(campitch));
    dirY=sinf(ret_radians(campitch));
    dirZ=sinf(ret_radians(camyaw))*cosf(ret_radians(campitch));
}
void draw_transparent_cube(float size) {
    float h = size / 2.0f; 
    glColor4f(0.5f, 0.5f, 1.0f, 0.3f); 
    glBegin(GL_QUADS);
    glVertex3f(-h, -h,  h); glVertex3f( h, -h,  h);
    glVertex3f( h,  h,  h); glVertex3f(-h,  h,  h);
    glVertex3f(-h, -h, -h); glVertex3f( h, -h, -h);
    glVertex3f( h,  h, -h); glVertex3f(-h,  h, -h);
    glVertex3f(-h, -h, -h); glVertex3f(-h, -h,  h);
    glVertex3f(-h,  h,  h); glVertex3f(-h,  h, -h);
    glVertex3f( h, -h, -h); glVertex3f( h, -h,  h);
    glVertex3f( h,  h,  h); glVertex3f( h,  h, -h);
    glVertex3f(-h,  h, -h); glVertex3f(-h,  h,  h);
    glVertex3f( h,  h,  h); glVertex3f( h,  h, -h);
    glVertex3f(-h, -h, -h); glVertex3f(-h, -h,  h);
    glVertex3f( h, -h,  h); glVertex3f( h, -h, -h);
    glEnd();
}

void reshape(int w, int h){
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0,(float)w/h,0.1,100.0);
    glMatrixMode(GL_MODELVIEW);
}
void draw_boid(Boid*b){
    vec3 pos=b->ret_position();
    vec3 velo=b->ret_velocity();
    vec3 acc=b->ret_acceleration();
    if(length(velo)<0.01f){
        velo = normalize(velo+vec3(0.1f,0,0));
    }
    vec3 norm_dir=normalize(velo);
    vec3 up = vec3(0, 0, 1);
    if (abs(dot(norm_dir, up)) > 0.99f) {
        up = vec3(0, 1, 0);
    }
    vec3 right = normalize(cross(norm_dir, up));
    vec3 perp = normalize(cross(right, norm_dir));
    float size=0.2f;
    vec3 tip=pos+norm_dir*size;
    vec3 base_center=pos-norm_dir*size*0.6f;
    vec3 base_right = base_center + right * size * 0.5f;
    vec3 base_left  = base_center - right * size * 0.5f;
    glBegin(GL_TRIANGLES);
    glColor3f(0, 0, 0); 
    glVertex3f(tip.x, tip.y, tip.z);
    glVertex3f(base_right.x, base_right.y, base_right.z);
    glVertex3f(base_left.x, base_left.y, base_left.z);
    glEnd();
}
vec3 norm(vec3 pos){
    float len = length(pos);
    if(len<1e-6) return vec3(0);
    return pos/length(pos);
}
vector<Boid*>get_neighbors(Boid*specific_boid){
    //currently this is dealing in 2d, so will use a circle that is trimmed off.
    //so we can take the velocity vector, as unit vector as the normal.So the angle is aroudn this.
    float v_x=specific_boid->ret_velocity().x;
    float v_y=specific_boid->ret_velocity().y;
    float v_z=specific_boid->ret_velocity().z;
    float norm_det=sqrt(v_x*v_x+v_y*v_y+v_z*v_z);
    vec3 vel_unit=specific_boid->ret_velocity();
    vec3 unit_vector;
    if(norm_det<1e-6){
        unit_vector=vec3(1,0,0);
    }
    else{
        unit_vector=vel_unit/norm_det;
    }
    //we will check all of the neighbours other that this, and angle around thsi normal.
    vector<Boid*>ret;
    for(auto &k: all_boids){
        if(k!=specific_boid){
            float dist =length(k->ret_position()-specific_boid->ret_position());
            vec3 dir_vec = k->ret_position()-specific_boid->ret_position();
            float angle=acos(std::clamp(dot(normalize(dir_vec),unit_vector),-1.0f,1.0f));
            if(angle<= specific_boid->vision_angle*0.5f && dist<=specific_boid->vision_radius){
                ret.push_back(k);
            }
        }
    }
    return ret;
}
vec3 compute_seperation(Boid*specific_boid,vector<Boid*>&neighbors){
    //this is the sterring acc suggestion from the seperation of all neighbours.
    //the idea is that we will avoid the incoming neighbouring partices only if they are already inside the safe_bubbel.
    vec3 steer = vec3(0);
    int cnt=0;
    for(auto &nei: neighbors){
        float d=length(specific_boid->ret_position()-nei->ret_position());
        if(d<specific_boid->safe_bubble && d>1e-6){
            vec3 diff=specific_boid->ret_position()-nei->ret_position();
            diff=normalize(diff)*(1.0f/std::max(d,0.3f));//currently scaled by inverse of the weight.
            steer+=diff;
            cnt++;
        }
    }
    if(cnt>0){  
        steer=steer/(float)cnt;
    }
    if(length(steer)>0){
        steer=normalize(steer)*specific_boid->max_speed-specific_boid->ret_velocity();
        if(length(steer)>specific_boid->max_force){
            steer=normalize(steer)*specific_boid->max_force;
        }
    }
    return steer*specific_boid->sep_weight;
}
vec3 compute_mean_vel_steer(Boid *specific_boid,vector<Boid*>&neighbors){
    //steering suggestion(acc) for mean velocity of the flock.
    vec3 ret=vec3(0);
    int cnt=0;
    for(auto &nei:neighbors){
        ret+= nei->ret_velocity();
        cnt++;
    }
    if(cnt==0) return vec3(0);
    vec3 mean_vel= ret/(float)cnt;
    vec3 steer=normalize(mean_vel)*specific_boid->max_speed-specific_boid->ret_velocity();
    if(length(steer)>specific_boid->max_force){
        steer=normalize(steer)*specific_boid->max_force;
    }
    return steer*specific_boid->align_weight;
}
vec3 compute_cohesion(Boid* specific_boid,vector<Boid*>&neighbors){
    //to get it to follow the center of all the nearby flock.
    vec3 mid=vec3(0);
    int cnt=0;
    for(auto &nei:neighbors){
        mid +=nei->ret_position();
        cnt++;
    }
    if(cnt==0){
        return vec3(0);
    }
    mid=mid/(float)cnt;
    vec3 req=mid-specific_boid->ret_position();
    if(length(req)>0){
        req=normalize(req)*specific_boid->max_speed;
    }
    vec3 steer_acc=req-specific_boid->ret_velocity();
    if(length(steer_acc)>specific_boid->max_force){
        steer_acc=normalize(steer_acc)*specific_boid->max_force;
    }
    return steer_acc*specific_boid->cohesion_weight;
}
vec3 compute_obstacle_avoidance(Boid*b) {
    vec3 total=vec3(0);
    int count =0;
    for (auto obs:all_obs) {
        if (obs->isviewable(b)) {
            vec3 f=obs->avoid_force(b);
            total+=f;
            count++;
        }
    }
    if(count==0)return vec3(0);
    total/=float(count);
    total*=b->obs_avoid_weight;
    if (length(total)>b->max_force){
        total=normalize(total)*b->max_force;
    }
    return total;
}
void update_boid(Boid *specific_boid){
    vector<Boid*>neighbors=get_neighbors(specific_boid);
    vec3 sep=compute_seperation(specific_boid,neighbors);
    vec3 ali=compute_mean_vel_steer(specific_boid,neighbors);
    vec3 coh=compute_cohesion(specific_boid,neighbors);
    vec3 obj_mov=compute_obstacle_avoidance(specific_boid);
    vec3 acc=sep+ali+coh+obj_mov;
    if(length(acc)>specific_boid->max_acc){
        acc = normalize(acc)*specific_boid->max_acc;
    }
    specific_boid->apply_force(acc);
    specific_boid->update();
}
void update(int val){
    for(Boid *b:all_boids){
        update_boid(b);
    }
    glutPostRedisplay();
    glutTimerFunc(16,update,0);//this is 60fps.
}
void trial_draw_boid()
{
    glBegin(GL_TRIANGLES);
    glColor3f(0, 0, 0); 
    glVertex3f(0.0f, 0.2f, 0.0f); 
    glVertex3f(-0.2f, -0.2f, 0.0f); 
    glVertex3f(0.2f, -0.2f, 0.0f); 
    glEnd();
}
void display(){
 glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
 glLoadIdentity();
 float dirX,dirY,dirZ;
 getCameraDir(dirX,dirY,dirZ);
 gluLookAt(
        camX,camY,camZ,camX+dirX,camY+dirY,camZ+dirZ,0,1,0);
 glBegin(GL_LINES);
        glColor3f(1,0,0);glVertex3f(0,0,0);glVertex3f(2,0,0); //X
        glColor3f(0,1,0);glVertex3f(0,0,0);glVertex3f(0,2,0); //Y
        glColor3f(0,0,1);glVertex3f(0,0,0);glVertex3f(0,0,2); //Z
 glEnd();
 glPushMatrix();
 for(auto &obs : all_obs) {
    obs->draw();
    }
 for(auto &b: all_boids){
        draw_boid(b);
    }
 glPopMatrix();
 glPushMatrix();
    draw_transparent_cube(10.0f);
 glPopMatrix();
 glutSwapBuffers();
}

void init(){
    glEnable(GL_DEPTH_TEST);
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}
void mouseButton(int button,int state,int x,int y){
    if(button==GLUT_LEFT_BUTTON){
        leftDown = (state==GLUT_DOWN);
    }
    if(button==GLUT_RIGHT_BUTTON){
        rightDown=(state==GLUT_DOWN);
    }
    lastMouseX=x;lastMouseY=y;
}
void MouseMotion(int x,int y){
    float dx=x-lastMouseX;
    float dy=y-lastMouseY;
    if(leftDown){ //this is for yaw/pitch rot.
        camyaw += dx*sens;
        campitch-= dy*sens;
        if(campitch>89.0f) campitch=89.0f;
        if(campitch < -89.0f) campitch=-89.0f;
    }
    if(rightDown){
        //this is for panning the cam.
        camX -= dx*0.01f;
        camY -= dy*0.01;
    }
    lastMouseX=x;
    lastMouseY=y;
    glutPostRedisplay();
}
void keyboard(unsigned char key,int x,int y){
    float dirX,dirY,dirZ;
    getCameraDir(dirX,dirY,dirZ);
    float len=sqrt(dirX*dirX+dirY*dirY+dirZ*dirZ);
    dirX/=len;
    dirY/=len;
    dirZ /= len;
    float rightX = dirZ;
    float rightZ=-dirX;
    switch(key){
    case 'w':
        camX+=dirX*speed;
        camY+=dirY*speed;
        camZ+=dirZ*speed;
        break;  
    case 's':
        camX-=dirX*speed;
        camY-=dirY*speed;
        camZ-=dirZ*speed;
        break;
    case 'a':
        camX-=rightX*speed;
        camZ-=rightZ*speed;
        break;
    case 'd':
        camX+=rightX*speed;
        camZ+=rightZ*speed;
        break;
    case 'z':camZ-=0.3f; break;
    case 'x':camZ+=0.3f; break; 
    case 'q':camY-=speed; break;
    case 'e':camY+=speed; break;
    case 27:exit(0);
    }
    glutPostRedisplay();
}
void specialKeys(int key,int x,int y) {
    if(key==GLUT_KEY_LEFT){
        camyaw-=2.0;
    }
    if(key==GLUT_KEY_RIGHT){
        camyaw+=2.0;
    }
    if(key==GLUT_KEY_UP){
        campitch+=2.0;
    }
    if(key==GLUT_KEY_DOWN){
        campitch-=2.0;
    }
    glutPostRedisplay();
}
void init_boids(int n){
    for(int i=0;i<n;i++){
        float x = ((float)rand() / RAND_MAX) * 10.0f - 5.0f;
        float y = ((float)rand() / RAND_MAX) * 10.0f - 5.0f;
        float z = ((float)rand() / RAND_MAX) * 10.0f - 5.0f;
        vec3 pos{x,y,z};
        Boid* boid_pointer=new Boid(pos);
        all_boids.push_back(boid_pointer);
    }
}
void myGlutIdle(void) {
    if (main_window > 0) {   
        glutSetWindow(main_window);
        glutPostRedisplay();
    }
}
int main(int argc,char **argv){
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
    glutInitWindowSize(1000,800);
    main_window = glutCreateWindow("boid simulation");
    init();
    init_boids(100);
    glutDisplayFunc(display);
    glutIdleFunc(display);
    glutMouseFunc(mouseButton);
    glutMotionFunc(MouseMotion);
    glutSpecialFunc(specialKeys);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);
    glutTimerFunc(16,update,0);
    all_obs.push_back(new sphere_obs(vec3(0, 0, 0), 1.5f));
    all_obs.push_back(new sphere_obs(vec3(3, 2, -2), 1.0f));
    all_obs.push_back(new sphere_obs(vec3(-4, -1, 1), 2.0f));
    GLUI *glui = GLUI_Master.create_glui("Controls");
    GLUI_Spinner *visible_spinner = new GLUI_Spinner(glui, "Visible Radius", &g_vision_radius);
    visible_spinner->set_float_limits(0.1f, 2.5f);
    GLUI_Spinner *safe_spinner = new GLUI_Spinner(glui, "Safe Radius", &g_safe_bubble);
    safe_spinner->set_float_limits(0.1f, 2.5f);
    GLUI_Spinner *align_spinner = new GLUI_Spinner(glui, "Alignment Weight", &g_align_weight);
    align_spinner->set_float_limits(0.0f, 5.0f);
    GLUI_Spinner *cohesion_spinner = new GLUI_Spinner(glui, "Cohesion Weight", &g_cohesion_weight);
    cohesion_spinner->set_float_limits(0.0f, 5.0f);
    GLUI_Spinner *separation_spinner = new GLUI_Spinner(glui, "Separation Weight", &g_separation_weight);
    separation_spinner->set_float_limits(0.0f, 5.0f);
    glui->set_main_gfx_window(main_window);
    GLUI_Master.set_glutIdleFunc(myGlutIdle);
    glutMainLoop();
    return 0;
}
