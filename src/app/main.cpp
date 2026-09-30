#include "rescueswarm/simulation/simulation_clock.hpp"
#include "rescueswarm/simulation/world.hpp"
#include "rescueswarm/telemetry/metrics.hpp"
#ifdef RESCUESWARM_WITH_RENDERER
#include "rescueswarm/rendering/renderer.hpp"
#endif
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace { struct Options { std::string scenario="scenarios/city.json"; unsigned seed=42; int benchmark=0; bool headless=false; };
Options parse(int argc,char**argv){Options o;for(int i=1;i<argc;++i){std::string a=argv[i];if(a=="--scenario"&&i+1<argc)o.scenario=argv[++i];else if(a=="--seed"&&i+1<argc)o.seed=static_cast<unsigned>(std::stoul(argv[++i]));else if(a=="--benchmark"&&i+1<argc)o.benchmark=std::stoi(argv[++i]);else if(a=="--headless")o.headless=true;else if(a=="--help"){std::cout<<"RescueSwarm 3D\n  --scenario FILE  --seed N  --benchmark N  --headless\n";std::exit(0);}}return o;} }
int main(int argc,char**argv){try{auto o=parse(argc,argv);if(!std::filesystem::exists(o.scenario)){auto executable=std::filesystem::weakly_canonical(argv[0]);auto bundled=executable.parent_path().parent_path()/o.scenario;if(std::filesystem::exists(bundled))o.scenario=bundled.string();}auto scenario=rs::Scenario::load(o.scenario);if(o.benchmark>0){rs::AggregateMetrics total;for(int i=0;i<o.benchmark;++i){rs::World world(scenario,o.seed+static_cast<unsigned>(i));while(!world.finished())world.tick(1.0f/30.0f);total.add(world.metrics());}total.print(std::cout);return 0;}rs::World world(scenario,o.seed);if(o.headless){while(!world.finished())world.tick(1.0f/30.0f);rs::AggregateMetrics total;total.add(world.metrics());total.print(std::cout);return 0;}
#ifdef RESCUESWARM_WITH_RENDERER
rs::Renderer renderer(1440,900);rs::SimulationClock clock;while(!renderer.shouldClose()){clock.advance(renderer.frameTime(),world.time_scale,[&](float dt){world.tick(dt);});renderer.draw(world,o.scenario,o.seed);}return 0;
#else
throw std::runtime_error("This binary was built without rendering support. Reconfigure with RESCUESWARM_BUILD_RENDERER=ON.");
#endif
}catch(const std::exception&e){std::cerr<<"error: "<<e.what()<<'\n';return 1;}}
