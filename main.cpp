#include <list>

#include <SDL.h>
#include <SDL_render.h>
#include <imgui.h>
#include <imgui_sdl.h>
#include <imgui_impl_sdl.h>

#if __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "algorithm.hpp"

template<class T>
class Solver
{
public:
  Solver() = default;
  ~Solver() = default;

  void add_point(T x, T y) { point_list.push_back(wykobi::make_point(x, y)); }

  void generate_points(T x, T y, T width, T height, std::size_t max_points)
  {
    point_list.clear();
    point_list.reserve(point_list.size() + max_points);
    wykobi::generate_random_points<T>(
      x, y, x + width, y + height, max_points, std::back_inserter(point_list));
  }
  void solve()
  {
    solve_quick_hull(
      point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
  }

  template<typename InputIterator, typename OutputIterator>
  void solve_quick_hull(InputIterator begin,
                        InputIterator end,
                        OutputIterator out)
  {
    debug.first_horizontal_split[0].x = 0;
    debug.first_horizontal_split[1].x = 0;
    debug.first_horizontal_split[0].y = 0;
    debug.first_horizontal_split[1].y = 0;
    debug.split_triangles.clear();
    convex_hull.clear();
    algorithm::convex_hull_quick_hull<wykobi::point2d<T>>(
      begin, end, out, debug);
  }

  template<typename InputIterator, typename OutputIterator>
  void solve_graham_scan(InputIterator begin,
                         InputIterator end,
                         OutputIterator out)
  {
    convex_hull.clear();
    wykobi::algorithm::convex_hull_graham_scan<wykobi::point2d<T>>(
      begin, end, out);
  }

  uint32_t num_points_to_generate = 10;

private:
  std::vector<wykobi::point2d<T>> point_list;
  wykobi::polygon<T, 2> convex_hull;

  typename algorithm::convex_hull_quick_hull<wykobi::point2d<T>>::debug_data_t
    debug;

  friend class Renderer;
};

class Renderer
{
  struct options_t
  {
    bool show_first_horizontal_split = true;
    bool show_first_convex_hull_vertex = true;
    bool show_last_convex_hull_edge = true;
    int show_split_triangle = -1;
  };

public:
  Renderer()
    : window(nullptr)
    , renderer(nullptr)
    , ui(nullptr)
  {}
  ~Renderer() { terminate(); }

  bool initialize()
  {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
      std::printf("Failed to initialize SDL\n");
      return false;
    }

    window = SDL_CreateWindow(
      "quickhull", SDL_HINT_DEFAULT, SDL_HINT_DEFAULT, 800, 600, 0);
    if (!window) {
      std::printf("Failed to create SDL window\n");
      terminate();
      return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
      std::printf("Failed to create SDL renderer\n");
      terminate();
      return false;
    }

    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    width = w;
    height = h;

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ui = ImGui::CreateContext();
    if (!ui) {
      std::printf("Failed to create ImGui context\n");
      terminate();
      return false;
    }

    // Setup Platform/Renderer bindings
    ImGuiSDL::Initialize(renderer, width, height);
    ImGui_ImplSDL2_InitForOpenGL(window, nullptr);

    return true;
  }

  void terminate() noexcept
  {
    if (ui) {
      ImGuiSDL::Deinitialize();
      ImGui_ImplSDL2_Shutdown();
      ImGui::DestroyContext(ui);
      ui = nullptr;
    }

    if (renderer) {
      SDL_DestroyRenderer(renderer);
      renderer = nullptr;
    }
    if (window) {
      SDL_DestroyWindow(window);
      window = nullptr;
    }
    SDL_Quit();
  }

  template<class T>
  void draw(Solver<T>& solver)
  {
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();

    ImGui::Begin("Controls");
    ImGui::Text("Keyboard controls:\n\tSpacebar: Regenerate Set.\n\tBackspace: "
                "Clear all points.\n"
                "Mouse: Click to add points to set.");
    ImGui::Checkbox("Show first horizontal split",
                    &debug_options.show_first_horizontal_split);
    ImGui::Checkbox("Show first convex hull vertex",
                    &debug_options.show_first_convex_hull_vertex);
    ImGui::Checkbox("Show last convex hull edge",
                    &debug_options.show_last_convex_hull_edge);
    ImGui::SliderInt("Show split triangle",
                     &debug_options.show_split_triangle,
                     -1,
                     solver.debug.split_triangles.size() - 1);

    ImGui::InputScalar("Amount of points to generate",
                       ImGuiDataType_U32,
                       &solver.num_points_to_generate);
    if (ImGui::Button("Generate Random point set")) {
      solver.generate_points(width * 0.25f,
                             height * 0.25f,
                             width * 0.5f,
                             height * 0.5f,
                             solver.num_points_to_generate);
      solver.solve();
    }
    ImGui::End();

    const SDL_Rect clip = { 0, 0, static_cast<int>(width), static_cast<int>(height) };
    SDL_RenderSetClipRect(renderer, &clip);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 127, 127, 127, SDL_ALPHA_OPAQUE);
    draw(solver.point_list);
    draw(solver.convex_hull);
    draw(solver.debug);

    ImGui::Render();
    ImGuiSDL::Render(ImGui::GetDrawData());

    SDL_RenderPresent(renderer);
  }

  template<class T>
  void draw(const wykobi::polygon<T, 2>& polygon)
  {
    if (polygon.size() == 0) {
      return;
    }
    for (uint32_t i = 0; i < polygon.size() - 1; ++i) {
      SDL_RenderDrawLine(renderer,
                         polygon[i].x,
                         height - polygon[i].y,
                         polygon[i + 1].x,
                         height - polygon[i + 1].y);
    }
    if (debug_options.show_last_convex_hull_edge) {
      SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    }
    if (polygon.size() > 1) {
      SDL_RenderDrawLine(renderer,
                         polygon[polygon.size() - 1].x,
                         height - polygon[polygon.size() - 1].y,
                         polygon[0].x,
                         height - polygon[0].y);
    } else if (polygon.size() == 1) {
      SDL_RenderDrawPoint(renderer, polygon[0].x, height - polygon[0].y);
    }

    if (debug_options.show_first_convex_hull_vertex) {
      constexpr int width = 10;
      if (polygon.size() >= 1) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 0, SDL_ALPHA_OPAQUE);
        SDL_Rect rect{
          static_cast<int>(polygon[0].x - width / 2),
          static_cast<int>(height - polygon[0].y - width / 2),
          width,
          width,
        };
        SDL_RenderDrawRect(renderer, &rect);
      }
    }
  }

  template<class T>
  void draw(const std::vector<wykobi::point2d<T>>& point_list)
  {
    for (const auto& point : point_list) {
      SDL_RenderDrawPoint(renderer, point.x, height - point.y);
    }
  }

  void draw(const algorithm::convex_hull_quick_hull<
            wykobi::point2d<double>>::debug_data_t& debug)
  {
    if (debug_options.show_first_horizontal_split) {
      SDL_SetRenderDrawColor(renderer, 0, 0, 255, SDL_ALPHA_OPAQUE);
      SDL_RenderDrawLine(renderer,
                         debug.first_horizontal_split[0].x,
                         height - debug.first_horizontal_split[0].y,
                         debug.first_horizontal_split[1].x,
                         height - debug.first_horizontal_split[1].y);
    }
    if (debug_options.show_split_triangle >= debug.split_triangles.size()) {
      debug_options.show_split_triangle = -1;
    }
    if (debug_options.show_split_triangle >= 0) {
      SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
      auto& t = debug.split_triangles[debug_options.show_split_triangle];

      std::string point_names[3] = { "P", "C", "Q" };
      for (uint8_t i = 0; i < 3; ++i) {
        ImGui::SetNextWindowPos(ImVec2(t[i].x, height - t[i].y));
        ImGui::Begin(point_names[i].c_str(),
                     nullptr,
                     ImGuiWindowFlags_NoBackground |
                       ImGuiWindowFlags_NoDecoration |
                       ImGuiWindowFlags_NoInputs);
        ImGui::Text("%s", point_names[i].c_str());
        ImGui::End();
      }

      SDL_RenderDrawLine(
        renderer, t[0].x, height - t[0].y, t[1].x, height - t[1].y);
      SDL_RenderDrawLine(
        renderer, t[1].x, height - t[1].y, t[2].x, height - t[2].y);
      SDL_RenderDrawLine(
        renderer, t[2].x, height - t[2].y, t[0].x, height - t[0].y);
    }
  }

  void resolution(uint32_t& out_width, uint32_t& out_height) const
  {
    out_width = width;
    out_height = height;
  }

  bool ui_want_capture_mouse() const
  {
    ImGuiIO& io = ImGui::GetIO();
    return io.WantCaptureMouse;
  }

  bool ui_want_capture_keyboard() const
  {
    ImGuiIO& io = ImGui::GetIO();
    return io.WantCaptureKeyboard;
  }


private:
  SDL_Window* window;
  SDL_Renderer* renderer;
  ImGuiContext* ui;
  options_t debug_options;
  uint32_t width;
  uint32_t height;
};

#if __EMSCRIPTEN__
void
em_main_loop_callback(void* arg);
#endif

class App
{
public:
  App() = default;
  virtual ~App() = default;

  bool main_loop()
  {
    renderer.draw(solver);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      uint32_t width, height;
      renderer.resolution(width, height);
      if (event.type == SDL_QUIT) {
        return false;
      } else if (event.type == SDL_MOUSEBUTTONDOWN && !renderer.ui_want_capture_mouse()) {
        solver.add_point(event.button.x, height - event.button.y);
        solver.solve();
      } else if (event.type == SDL_KEYDOWN && !renderer.ui_want_capture_keyboard()) {
        switch (event.key.keysym.scancode) {
          case SDL_SCANCODE_SPACE:
            solver.generate_points(width * 0.25f,
                                   height * 0.25f,
                                   width * 0.5f,
                                   height * 0.5f,
                                   solver.num_points_to_generate);
            solver.solve();
            break;
          case SDL_SCANCODE_BACKSPACE:
            solver.generate_points(0, 0, 0, 0, 0);
            solver.solve();
            break;
          case SDL_SCANCODE_ESCAPE:
            return false;
        }
      }
    }
    return true;
  }

  int run()
  {
    if (!renderer.initialize()) {
      return EXIT_FAILURE;
    }

    uint32_t width, height;
    renderer.resolution(width, height);
    solver.generate_points(width * 0.25f,
                           height * 0.25f,
                           width * 0.5f,
                           height * 0.5f,
                           solver.num_points_to_generate);
    solver.solve();

#if __EMSCRIPTEN__
    emscripten_set_main_loop_arg(em_main_loop_callback, this, 0, true);
#else
    while (main_loop()) {
    }
#endif

    terminate();

    return EXIT_SUCCESS;
  }

  void terminate() { renderer.terminate(); }

private:
  Renderer renderer;
  Solver<double> solver;
};

#if __EMSCRIPTEN__
void
em_main_loop_callback(void* arg)
{
  auto app = reinterpret_cast<App*>(arg);
  if (!app->main_loop()) {
    app->terminate();
    emscripten_cancel_main_loop();
  }
}
#endif

int main(int argc, char* argv[])
{
  App app;
  return app.run();
}
