#include "rescueswarm/rendering/renderer.hpp"
#include "rescueswarm/simulation/world.hpp"
#include "imgui.h"
#include "raylib.h"
#include "rlImGui.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

namespace rs {
namespace {
Vector3 rv(const Vec3& v) { return {v.x(), v.y(), v.z()}; }
Color droneColor(int id) { static Color c[]={{55,210,255,255},{255,166,48,255},{80,235,130,255},{235,75,255,255},{255,222,70,255},{255,100,145,255}}; return c[id%6]; }
Color stateColor(DroneState s) { switch(s){case DroneState::Investigate:case DroneState::ConfirmSurvivor:return GOLD;case DroneState::ReturnToBase:return ORANGE;case DroneState::Land:return LIME;case DroneState::Failed:return RED;default:return SKYBLUE;} }

void drawRoads(){const Color asphalt{25,31,40,255};DrawCube({20,.035f,20},4.2f,.07f,40,asphalt);DrawCube({20,.04f,20},40,.08f,4.2f,asphalt);DrawCube({10,.045f,20},1.8f,.09f,40,asphalt);DrawCube({30,.045f,20},1.8f,.09f,40,asphalt);for(int i=1;i<40;i+=3){DrawCube({20,.09f,(float)i},.11f,.02f,1.25f,Fade(GOLD,.65f));DrawCube({(float)i,.095f,20},1.25f,.02f,.11f,Fade(GOLD,.65f));}}

void drawBuilding(const Eigen::AlignedBox3f& box,int index){
    Vec3 size=box.max()-box.min(),center=(box.min()+box.max())*.5f;unsigned char shade=(unsigned char)(43+(index%4)*8);Color wall{shade,(unsigned char)(shade+12),(unsigned char)(shade+23),255};
    DrawCube(rv(center),size.x(),size.y(),size.z(),wall);DrawCubeWires(rv(center),size.x(),size.y(),size.z(),{112,139,160,255});Color window{35,155,205,190};
    for(float y=1.6f;y<box.max().y()-.5f;y+=1.65f){for(float z=box.min().z()+1.1f;z<box.max().z()-.5f;z+=1.8f)DrawCube({box.max().x()+.015f,y,z},.035f,.65f,.72f,window);for(float x=box.min().x()+1.1f;x<box.max().x()-.5f;x+=1.8f)DrawCube({x,y,box.max().z()+.015f},.72f,.65f,.035f,window);}
    float roof=box.max().y()+.06f;DrawLine3D({box.min().x(),roof,box.min().z()},{center.x(),roof-.45f,center.z()},Fade(ORANGE,.8f));DrawLine3D({box.max().x(),roof,box.max().z()},{center.x(),roof-.45f,center.z()},Fade(ORANGE,.8f));
    for(int r=0;r<5;++r){float side=r%2==0?-1.f:1.f;Vec3 rubble{center.x()+side*(size.x()*.55f+.35f*r),.22f+.08f*(r%3),center.z()+(((r*7+index*3)%9)/9.f-.5f)*size.z()};DrawCube(rv(rubble),.55f+.12f*(r%2),.4f,.6f,Fade(wall,.95f));DrawCubeWires(rv(rubble),.55f+.12f*(r%2),.4f,.6f,Fade(LIGHTGRAY,.5f));}
}

void drawDrone3D(const Drone& d,bool sensors,bool paths){
    Color color=droneColor(d.id);Vec3 p=d.position;float pulse=.5f+.5f*std::sin((float)GetTime()*8.f+d.id);Vec3 a{.62f,0,.62f},b{.62f,0,-.62f};DrawLine3D(rv(p-a),rv(p+a),color);DrawLine3D(rv(p-b),rv(p+b),color);DrawSphere(rv(p),.32f,color);DrawSphereWires(rv(p),.43f,8,8,Fade(WHITE,.9f));
    const Vec3 offsets[]={a,Vec3(-a),b,Vec3(-b)};for(const Vec3& off:offsets){Vec3 rotor=p+off;DrawCylinder(rv(rotor),.27f,.27f,.045f,18,Fade(WHITE,.92f));DrawCircle3D(rv(rotor+Vec3{0,.035f,0}),.34f+.04f*pulse,{1,0,0},90,Fade(color,.65f));}
    DrawLine3D(rv(p),{p.x(),.12f,p.z()},Fade(color,.35f));DrawCircle3D({p.x(),.13f,p.z()},.65f,{1,0,0},90,Fade(color,.45f));
    if(sensors){DrawCircle3D(rv(p),d.sensor_range,{1,0,0},90,Fade(color,.22f));DrawCircle3D(rv(p),d.sensor_range,{0,0,1},90,Fade(color,.16f));}
    for(size_t i=1;i<d.trajectory.size();++i)DrawLine3D(rv(d.trajectory[i-1]),rv(d.trajectory[i]),Fade(color,.72f));
    if(paths){Vec3 prev=p;for(const auto& point:d.path){DrawLine3D(rv(prev),rv(point),Fade(color,.92f));DrawSphere(rv(point),.075f,Fade(color,.8f));prev=point;}if(d.target_active){DrawCylinder(rv(d.mission_target),.38f,.08f,.08f,16,color);DrawCircle3D(rv(d.mission_target),.62f,{1,0,0},90,Fade(color,.8f));}}
}
}

struct Renderer::Impl{
    Camera3D camera{};bool show_voxels=false,show_paths=true,show_links=true,show_sensors=true,show_labels=true;const char* capture_path=nullptr;int rendered_frames=0,capture_after=60;bool capture_complete=false;
    Impl(){SetConfigFlags(FLAG_MSAA_4X_HINT|FLAG_VSYNC_HINT|FLAG_WINDOW_RESIZABLE);InitWindow(1440,900,"RescueSwarm 3D - Multi-Agent SAR Autonomy");SetTargetFPS(120);camera.position={48,34,48};camera.target={20,2.5f,20};camera.up={0,1,0};camera.fovy=50;camera.projection=CAMERA_PERSPECTIVE;capture_path=std::getenv("RESCUESWARM_CAPTURE_FRAME");if(const char* f=std::getenv("RESCUESWARM_CAPTURE_AFTER"))capture_after=std::max(1,std::atoi(f));rlImGuiSetup(true);}~Impl(){rlImGuiShutdown();CloseWindow();}
};
Renderer::Renderer(int,int):impl_(std::make_unique<Impl>()){}Renderer::~Renderer()=default;bool Renderer::shouldClose()const{return WindowShouldClose()||impl_->capture_complete;}float Renderer::frameTime()const{return GetFrameTime();}

void Renderer::draw(World& w,const std::string&,unsigned){
    if(!ImGui::GetIO().WantCaptureMouse&&IsMouseButtonDown(MOUSE_BUTTON_RIGHT))UpdateCamera(&impl_->camera,CAMERA_FREE);auto metrics=w.metrics();int confirmed=metrics.survivors_found;
    BeginDrawing();ClearBackground({6,10,18,255});BeginMode3D(impl_->camera);DrawPlane({20,-.02f,20},{40,40},{19,27,37,255});drawRoads();for(int i=0;i<(int)w.scenario().obstacles.size();++i)drawBuilding(w.scenario().obstacles[i],i);
    Vec3 base=w.scenario().base;DrawCylinder({base.x(),.08f,base.z()},1.8f,1.8f,.15f,36,{25,130,170,255});DrawCircle3D({base.x(),.17f,base.z()},1.45f,{1,0,0},90,{80,230,255,255});DrawCircle3D({base.x(),.18f,base.z()},.75f,{1,0,0},90,WHITE);DrawLine3D({base.x(),.2f,base.z()},{base.x(),7,base.z()},Fade(SKYBLUE,.5f));
    for(size_t i=0;i<w.survivors().size();++i){const auto&s=w.survivors()[i];float pulse=.85f+.2f*std::sin((float)GetTime()*4.f+(float)i);Color c=s.confirmed?LIME:(s.detected?GOLD:RED);DrawSphere(rv(s.position),.42f,c);DrawSphereWires(rv(s.position),.62f*pulse,10,10,Fade(c,.85f));DrawCircle3D({s.position.x(),.12f,s.position.z()},.9f*pulse,{1,0,0},90,Fade(c,.7f));if(s.detected)DrawLine3D(rv(s.position),rv(s.position+Vec3{0,4,0}),Fade(c,.7f));}
    const auto& drones=w.drones();for(const auto&d:drones)drawDrone3D(d,impl_->show_sensors,impl_->show_paths);
    if(impl_->show_links)for(size_t i=0;i<drones.size();++i)for(size_t j=i+1;j<drones.size();++j)if(drones[i].comms_available&&drones[j].comms_available&&(drones[i].position-drones[j].position).norm()<drones[i].communication_radius){DrawLine3D(rv(drones[i].position),rv(drones[j].position),Fade(SKYBLUE,.5f));DrawSphere(rv((drones[i].position+drones[j].position)*.5f),.08f,SKYBLUE);}
    if(impl_->show_voxels){auto dim=w.known().dimensions();for(int y=1;y<dim.y();++y)for(int z=0;z<dim.z();++z)for(int x=0;x<dim.x();++x){Cell c{x,y,z};if(w.known().at(c)==Occupancy::Occupied)DrawCubeWires(rv(w.known().cellCenter(c)),.94f,.94f,.94f,Fade(ORANGE,.28f));}}
    EndMode3D();
    if(impl_->show_labels)for(const auto&d:drones){Vector2 s=GetWorldToScreen(rv(d.position+Vec3{0,1.05f,0}),impl_->camera);if(s.x>0&&s.x<GetScreenWidth()&&s.y>0&&s.y<GetScreenHeight()){std::string label="DR-0"+std::to_string(d.id)+"  "+stateName(d.state);int width=MeasureText(label.c_str(),15);DrawRectangle((int)s.x-5,(int)s.y-3,width+10,21,Fade(BLACK,.78f));DrawRectangle((int)s.x-5,(int)s.y-3,3,21,droneColor(d.id));DrawText(label.c_str(),(int)s.x,(int)s.y,15,RAYWHITE);}}
    DrawRectangle(GetScreenWidth()-330,18,312,82,Fade(BLACK,.78f));DrawRectangle(GetScreenWidth()-330,18,5,82,confirmed==metrics.survivor_count?LIME:SKYBLUE);DrawText("LIVE AUTONOMY",GetScreenWidth()-312,29,18,SKYBLUE);DrawText(TextFormat("SURVIVORS  %d / %d",confirmed,metrics.survivor_count),GetScreenWidth()-312,53,17,RAYWHITE);DrawText(TextFormat("MAP  %5.1f%%    T+%05.1fs",metrics.coverage*100.f,w.time()),GetScreenWidth()-312,76,15,LIGHTGRAY);
    rlImGuiBegin();ImGui::SetNextWindowPos({18,18},ImGuiCond_Always);ImGui::SetNextWindowSize({420,0},ImGuiCond_Always);ImGui::Begin("Mission Control",nullptr,ImGuiWindowFlags_NoSavedSettings|ImGuiWindowFlags_NoCollapse);ImGui::SetScrollX(0);ImGui::TextUnformatted(w.scenario().name.c_str());ImGui::TextColored({.35f,.85f,1,1},"COORDINATED FRONTIER SEARCH");ImGui::Separator();ImGui::Text("Mission time  %6.1f / %.0f s",w.time(),w.scenario().time_limit);ImGui::ProgressBar(metrics.coverage,{-1,0},"Mapped coverage");ImGui::Text("Survivors confirmed: %d / %d",confirmed,metrics.survivor_count);if(ImGui::Button(w.paused?"Resume":"Pause"))w.paused=!w.paused;ImGui::SameLine();ImGui::SetNextItemWidth(230);ImGui::SliderFloat("Time scale",&w.time_scale,.25f,8.f,"%.2fx");ImGui::Checkbox("Occupied voxels",&impl_->show_voxels);ImGui::SameLine();ImGui::Checkbox("Paths",&impl_->show_paths);ImGui::SameLine();ImGui::Checkbox("Labels",&impl_->show_labels);ImGui::Checkbox("Comms links",&impl_->show_links);ImGui::SameLine();ImGui::Checkbox("Sensor range",&impl_->show_sensors);ImGui::Separator();
    for(const auto&d:drones){ImGui::PushID(d.id);Color c=droneColor(d.id),status=stateColor(d.state);ImGui::TextColored({c.r/255.f,c.g/255.f,c.b/255.f,1},"DR-%02d",d.id);ImGui::SameLine();ImGui::TextColored({status.r/255.f,status.g/255.f,status.b/255.f,1},"%-17s",stateName(d.state));ImGui::SameLine();ImGui::Text("%5.1f%%",d.battery);ImGui::ProgressBar(d.battery/100.f,{-1,4},"");if(d.assigned_survivor>=0)ImGui::TextColored({1,.82f,.2f,1},"TASK: confirm survivor S-%02d",d.assigned_survivor);else ImGui::TextDisabled("TASK: map sector %d   |   replans %d",d.assigned_region+1,d.replans);ImGui::TextDisabled("GPS %s  COMMS %s  SENSOR %s",d.gps_available?"OK":"LOST",d.comms_available?"OK":"LOST",d.sensor_available?"OK":"LOST");ImGui::PopID();}
    ImGui::End();rlImGuiEnd();DrawText("Hold right mouse + WASD: free camera   |   Mission Control: overlays + simulation speed",18,GetScreenHeight()-28,16,GRAY);EndDrawing();
    ++impl_->rendered_frames;if(impl_->capture_path&&impl_->rendered_frames==impl_->capture_after){TakeScreenshot(impl_->capture_path);impl_->capture_complete=true;}
}
}
