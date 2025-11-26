#include "GLUT/glut.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cmath>
#include <algorithm>


using namespace std;
using namespace glm;

int windowWidth=800;
int windowHeight=800;
float worldSize=100.0f;
class Boid{
    private:
        vec3 position;
        vec3 velocity;
        vec3 acceleration; //this is what is to be applied after arbitration,of this timestep.
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
                            0.0f);
            if (length(velocity)<1e-6f){
                velocity = vec3(0.1f,0,0);
            }
            acceleration=vec3(0);
            max_speed=0.8f;
            max_acc=0.06f;
            max_force=0.1f;
            vision_radius=3.0f;
            vision_angle=radians(270.0f);
            safe_bubble= 2.2f;
            sep_weight=1.9f;
            align_weight=1.2f;
            cohesion_weight=0.8f;
            obs_avoid_weight=2.0f;
        }
        vec3 ret_position(){
            return this->position;
        }
        vec3 ret_velocity(){
            return this->velocity;
        }
        void apply_force(vec3 f) {
            acceleration+=f;
        }
        void update() {
            velocity+=acceleration;
            if (length(velocity)>max_speed)
                velocity=normalize(velocity)*max_speed;
            position+=velocity;
            if(position.x>100)position.x=-100;
            if(position.x<-100)position.x=100;
            if(position.y>100)position.y=-100;
            if(position.y<-100)position.y=100;
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
vector<Boid*>all_boids;//will be use to store all of the boids that are present.
vector<Obstacle*>all_obstacles;

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
    for (auto obs:all_obstacles) {
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
void draw_boid(Boid*b){
    vec3 pos=b->ret_position();
    vec3 velo=b->ret_velocity();
    if(length(velo)<0.01f){
        velo = normalize(velo+vec3(0.1f,0,0));
    }
    vec3 norm_dir=normalize(velo);
    vec3 perp_left{-norm_dir.y,norm_dir.x,0.0f};
    float size=2.0f;
    //trying to make the traingular boid.
    //  /_\ -> so size*dir up, and to the left and right will dp (size/2)*perp_dir.
    vec3 p1=pos+norm_dir*size;
    vec3 p2=pos-norm_dir*size+perp_left*size*0.5f;//this is the left one.
    vec3 p3=pos-norm_dir*size-perp_left*size*0.5f;
    glBegin(GL_TRIANGLES);
    glColor3f(1,1,0);
    glVertex3f(p1.x,p1.y,p1.z);
    glVertex3f(p2.x,p2.y,p2.z);
    glVertex3f(p3.x,p3.y,p3.z);
    glEnd();
}
void display(){
    glClear(GL_COLOR_BUFFER_BIT);
    for(auto obs:all_obstacles){
        obs->draw();
    }
    for(auto &b: all_boids){
        draw_boid(b);
    }
    glutSwapBuffers();
}
void update(int val){
    for(Boid *b:all_boids){
        update_boid(b);
    }
    glutPostRedisplay();
    glutTimerFunc(16,update,0);//this is 60fps.
}
void setup_opengl(){
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-worldSize,worldSize,-worldSize,worldSize);
    glMatrixMode(GL_MODELVIEW);
}
void init_boids(int n){
    for(int i=0;i<n;i++){
        float x=(rand()%200-100);
        float y=(rand()%200-100);
        vec3 pos{x,y,0.0f};
        Boid* boid_pointer=new Boid(pos);
        all_boids.push_back(boid_pointer);
    }
}
int main(int argc,char**argv){
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(windowWidth, windowHeight);
    all_obstacles.push_back(new circular_obs(vec3(10.0f,20.0f,0.0f),30.0f));
    //all_obstacles.push_back(new square_obs(vec3(50.0f,60.0f,0.0f),30.0f));
    glutCreateWindow("Boids Simulation");
    setup_opengl();
    init_boids(100);
    glutDisplayFunc(display);
    glutTimerFunc(16,update,0);
    glutMainLoop();
    return 0;
}