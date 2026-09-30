#pragma once
#include <memory>
#include <string>
namespace rs { class World; class Renderer { public: Renderer(int width, int height); ~Renderer(); bool shouldClose() const; float frameTime() const; void draw(World& world, const std::string& scenario_path, unsigned seed); private: struct Impl; std::unique_ptr<Impl> impl_; }; }

