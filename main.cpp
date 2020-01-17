#include <array>
#include <list>
#include <string_view>

#include <SDL.h>
#include <SDL_render.h>
#include <imgui.h>
#include <imgui_sdl.h>
#include <imgui_impl_sdl.h>
#include <chrono>

#if __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "algorithm.hpp"

enum class Algorithm
{
  // Reference
  GrahamScan,
  JarvisMarch,
  Melkman,

  // Ours
  QuickHull,
  ChansAlgorithm,

  _Count,
};

static constexpr std::array<std::string_view,
                            static_cast<size_t>(Algorithm::_Count)>
  algorithm_strings = {
    "Graham Scan (Reference)", "Jarvis March (Reference)",
    "Melkman (Reference)",     "Quickhull",
    "ChansAlgorithm",
  };

template<class T>
class Solver
{
public:
  Solver()
    : selected_algorithm(Algorithm::ChansAlgorithm)
  {}
  ~Solver() = default;

  void add_point(T x, T y) { point_list.push_back(wykobi::make_point(x, y)); }

  void add_collinear_points(const wykobi::rectangle<T>& rectangle,
                            std::size_t max_points)
  {
    wykobi::segment2d segment;
    segment[0] = wykobi::generate_random_point<T>(rectangle);
    segment[1] = wykobi::generate_random_point<T>(rectangle);
    generate_random_points(segment, max_points, std::back_inserter(point_list));
  }

  void add_near_collinear_points(const wykobi::rectangle<T>& rectangle,
                                 std::size_t max_points)
  {
    wykobi::rectangle<T> epsilon_rect =
      wykobi::make_rectangle<T>(-std::numeric_limits<T>::epsilon(),
                                -std::numeric_limits<T>::epsilon(),
                                std::numeric_limits<T>::epsilon(),
                                std::numeric_limits<T>::epsilon());
    wykobi::segment2d segment;
    segment[0] = wykobi::generate_random_point<T>(rectangle);
    segment[1] = wykobi::generate_random_point<T>(rectangle);
    for (std::size_t i = 0; i < max_points; ++i) {
      auto p = wykobi::generate_random_point<T>(segment);
      auto offset = wykobi::generate_random_point(epsilon_rect);
      point_list.push_back(wykobi::make_point(p.x + offset.x, p.y + offset.y));
    }
  }

  void generate_points(const wykobi::rectangle<T>& rectangle,
                       std::size_t max_points)
  {
    point_list.clear();
    point_list.reserve(point_list.size() + max_points);
    wykobi::generate_random_points<T>(
      rectangle, max_points, std::back_inserter(point_list));
  }
  void solve()
  {
    debug_quickhull.first_horizontal_split[0].x = 0;
    debug_quickhull.first_horizontal_split[1].x = 0;
    debug_quickhull.first_horizontal_split[0].y = 0;
    debug_quickhull.first_horizontal_split[1].y = 0;
    debug_quickhull.split_triangles.clear();
    debug_chans_algorithm.first_point.x = 0;
    debug_chans_algorithm.first_point.y = 0;
    debug_chans_algorithm.intermediate_subsets.clear();
    debug_chans_algorithm.intermediate_m.clear();
    debug_chans_algorithm.intermediate_hulls.clear();
    debug_chans_algorithm.intermediate_found_hull_points.clear();
    debug_chans_algorithm.intermediate_found_points.clear();
    convex_hull.clear();
    auto time_point = std::chrono::high_resolution_clock::now();
    switch (selected_algorithm) {
      case Algorithm::GrahamScan:
        solve_graham_scan(
            point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
        break;
      case Algorithm::JarvisMarch:
        solve_jarvis_march(
            point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
        convex_hull.reverse();
        break;
      case Algorithm::Melkman:
        solve_melkman(
            point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
        break;
      case Algorithm::QuickHull:
        solve_quickhull(
            point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
        break;
      case Algorithm::ChansAlgorithm:
        solve_chans_algorithm(point_list.cbegin(),
                              point_list.cend(),
                              std::back_inserter(convex_hull));
        break;
    }
    auto duration = std::chrono::high_resolution_clock::now() - time_point;
    using duration_format_t = std::chrono::duration<double, std::micro>;
    auto duration_us = std::chrono::duration_cast<duration_format_t>(duration);
    if (duration_us.count() < std::numeric_limits<double>::epsilon()) {
      std::printf(
        "%s took less time than the resolution of the clock for %zu points and "
        "%zu points on hull. Try adding more points.\n",
        algorithm_strings[static_cast<size_t>(selected_algorithm)].data(),
        point_list.size(),
        convex_hull.size());
    } else {
      auto rate = point_list.size() / duration_us.count() * 1000 * 1000;
      const char* prefix = "";
      constexpr std::array<std::string_view, 4> si_prefixes{
        "K", "M", "G", "T"
      };
      for (auto& i : si_prefixes) {
        if (rate < 1000) {
          break;
        }
        rate /= 1000;
        prefix = i.data();
      }
      std::printf(
        "%s took %.3f us for %zu points and %zu points on hull or %.2f%s "
        "points/s\n",
        algorithm_strings[static_cast<size_t>(selected_algorithm)].data(),
        duration_us.count(),
        point_list.size(),
        convex_hull.size(),
        rate,
        prefix);
    }
  }

  template<typename InputIterator, typename OutputIterator>
  void solve_quickhull(InputIterator begin,
                       InputIterator end,
                       OutputIterator out)
  {
    algorithm::convex_hull_quickhull<wykobi::point2d<T>>(
      begin, end, out, debug_quickhull);
  }

  template<typename InputIterator, typename OutputIterator>
  void solve_chans_algorithm(InputIterator begin,
                             InputIterator end,
                             OutputIterator out)
  {
    algorithm::convex_hull_chans_algorithm<wykobi::point2d<T>>(
      begin, end, out, debug_chans_algorithm);
  }

  template<typename InputIterator, typename OutputIterator>
  void solve_graham_scan(InputIterator begin,
                         InputIterator end,
                         OutputIterator out)
  {
    wykobi::algorithm::convex_hull_graham_scan<wykobi::point2d<T>>(
      begin, end, out);
  }

  template<typename InputIterator, typename OutputIterator>
  void solve_jarvis_march(InputIterator begin,
                          InputIterator end,
                          OutputIterator out)
  {
    wykobi::algorithm::convex_hull_jarvis_march<wykobi::point2d<T>>(
      begin, end, out);
  }

  template<typename InputIterator, typename OutputIterator>
  void solve_melkman(InputIterator begin,
                     InputIterator end,
                     OutputIterator out)
  {
    wykobi::algorithm::convex_hull_melkman<wykobi::point2d<T>>(begin, end, out);
  }

  int num_points_to_generate = 10;
  int num_collinear_points_to_generate = 3;
  int num_near_collinear_points_to_generate = 3;

private:
  std::vector<wykobi::point2d<T>> point_list;
  wykobi::polygon<T, 2> convex_hull;

  Algorithm selected_algorithm;

  typename algorithm::convex_hull_quickhull<wykobi::point2d<T>>::debug_data_t
    debug_quickhull;
  typename algorithm::convex_hull_chans_algorithm<
    wykobi::point2d<T>>::debug_data_t debug_chans_algorithm;

  friend class Renderer;
};

class Renderer
{
  struct options_t
  {
    bool show_first_convex_hull_vertex = true;
    bool show_last_convex_hull_edge = true;
    struct
    {
      bool show_first_horizontal_split = true;
      int show_split_triangle = -1;
    } quickhull;
    struct
    {
      bool show_first_point = true;
      int show_intermediate_iteration = -1;
      int show_intermediate_subset = -1;
      int show_intermediate_jarvis_found_points_iteration = -1;
    } chans_algorithm;
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
      "Convex Hull", SDL_HINT_DEFAULT, SDL_HINT_DEFAULT, 1280, 768, 0);
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

    auto item_current = algorithm_strings[static_cast<size_t>(solver.selected_algorithm)];
    if (ImGui::BeginCombo("Algorithm", item_current.data()))
    {
      for (int n = 0; n < algorithm_strings.size(); n++)
      {
        bool is_selected = (item_current == algorithm_strings[n]);
        if (ImGui::Selectable(algorithm_strings[n].data(), is_selected)) {
          solver.selected_algorithm = static_cast<Algorithm>(n);
          solver.solve();
        }
        if (is_selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }

    if (ImGui::Button("Generate Random point set")) {
      solver.generate_points(
        wykobi::make_rectangle<double>(
          width * 0.25f, height * 0.75f, width * 0.75f, height * 0.25f),
        solver.num_points_to_generate);
      solver.solve();
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::InputInt("##Amount of points to generate",
                    &solver.num_points_to_generate);
    ImGui::PopItemWidth();
    if (solver.num_points_to_generate < 1) {
      solver.num_points_to_generate = 1;
    }

    if (ImGui::Button("Add collinear points")) {
      solver.add_collinear_points(
        wykobi::make_rectangle<double>(
          width * 0.25f, height * 0.75f, width * 0.75f, height * 0.25f),
        solver.num_collinear_points_to_generate);
      solver.solve();
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::InputInt("##Add collinear point count",
                    &solver.num_collinear_points_to_generate);
    ImGui::PopItemWidth();
    if (solver.num_collinear_points_to_generate < 3) {
      solver.num_collinear_points_to_generate = 3;
    }

    if (ImGui::Button("Add near-collinear points")) {
      solver.add_near_collinear_points(
        wykobi::make_rectangle<double>(
          width * 0.25f, height * 0.75f, width * 0.75f, height * 0.25f),
        solver.num_near_collinear_points_to_generate);
      solver.solve();
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::InputInt("##Add near-collinear point count",
                    &solver.num_near_collinear_points_to_generate);
    ImGui::PopItemWidth();
    if (solver.num_near_collinear_points_to_generate < 3) {
      solver.num_near_collinear_points_to_generate = 3;
    }

    if (ImGui::Button("Clear point set")) {
      solver.generate_points(wykobi::make_rectangle<double>(0, 0, 0, 0), 0);
      solver.solve();
    }

    ImGui::Checkbox("Show first convex hull vertex",
                    &debug_options.show_first_convex_hull_vertex);
    ImGui::Checkbox("Show last convex hull edge",
                    &debug_options.show_last_convex_hull_edge);

    if (solver.selected_algorithm == Algorithm::QuickHull) {
      ImGui::Checkbox("Show first horizontal split",
                      &debug_options.quickhull.show_first_horizontal_split);
      ImGui::SliderInt("Show split triangle",
                       &debug_options.quickhull.show_split_triangle,
                       -1,
                       solver.debug_quickhull.split_triangles.size() - 1);
    } else if (solver.selected_algorithm == Algorithm::ChansAlgorithm) {
      ImGui::Checkbox("Show first selected convex hull vertex",
                      &debug_options.chans_algorithm.show_first_point);
      ImGui::SliderInt(
        "Show iteration details",
        &debug_options.chans_algorithm.show_intermediate_iteration,
        -1,
        solver.debug_chans_algorithm.intermediate_subsets.size() - 1);
      if (debug_options.chans_algorithm.show_intermediate_iteration >= 0 &&
          debug_options.chans_algorithm.show_intermediate_iteration <
            solver.debug_chans_algorithm.intermediate_subsets.size()) {
        ImGui::Text("expected output size = %u",
                    solver.debug_chans_algorithm
                      .intermediate_m[debug_options.chans_algorithm
                                        .show_intermediate_iteration]);
        ImGui::SliderInt(
          "Show intermediate subset",
          &debug_options.chans_algorithm.show_intermediate_subset,
          -1,
          solver.debug_chans_algorithm
              .intermediate_subsets[debug_options.chans_algorithm
                                      .show_intermediate_iteration]
              .size() -
            1);
        if (debug_options.chans_algorithm.show_intermediate_subset >= 0) {
          ImGui::Text(
            "size = %zu",
            solver.debug_chans_algorithm
              .intermediate_subsets
                [debug_options.chans_algorithm.show_intermediate_iteration]
                [debug_options.chans_algorithm.show_intermediate_subset]
              .size());
        }
        ImGui::SliderInt(
          "Show jarvis found found points iteration",
          &debug_options.chans_algorithm
             .show_intermediate_jarvis_found_points_iteration,
          -1,
          solver.debug_chans_algorithm
              .intermediate_found_hull_points[debug_options.chans_algorithm
                                                .show_intermediate_iteration]
              .size() -
            1);
      }
    }
    ImGui::End();

    const SDL_Rect clip = { 0, 0, static_cast<int>(width), static_cast<int>(height) };
    SDL_RenderSetClipRect(renderer, &clip);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    draw(solver.point_list);
    SDL_SetRenderDrawColor(renderer, 127, 127, 127, SDL_ALPHA_OPAQUE);
    draw(solver.convex_hull);
    if (solver.selected_algorithm == Algorithm::QuickHull) {
      draw(solver.debug_quickhull);
    } else if (solver.selected_algorithm == Algorithm::ChansAlgorithm) {
      draw(solver.debug_chans_algorithm);
    }

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

  void draw(const algorithm::convex_hull_quickhull<
            wykobi::point2d<double>>::debug_data_t& debug)
  {
    if (debug_options.quickhull.show_first_horizontal_split) {
      SDL_SetRenderDrawColor(renderer, 0, 0, 255, SDL_ALPHA_OPAQUE);
      SDL_RenderDrawLine(renderer,
                         debug.first_horizontal_split[0].x,
                         height - debug.first_horizontal_split[0].y,
                         debug.first_horizontal_split[1].x,
                         height - debug.first_horizontal_split[1].y);
    }
    if (debug_options.quickhull.show_split_triangle >=
        debug.split_triangles.size()) {
      debug_options.quickhull.show_split_triangle = -1;
    }
    if (debug_options.quickhull.show_split_triangle >= 0) {
      SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
      auto& t =
        debug.split_triangles[debug_options.quickhull.show_split_triangle];

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

  void draw(const algorithm::convex_hull_chans_algorithm<
            wykobi::point2d<double>>::debug_data_t& debug)
  {
    constexpr int highlight_box_width = 10;
    if (debug_options.chans_algorithm.show_first_point) {
      SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
      SDL_Rect rect{
        static_cast<int>(debug.first_point.x - highlight_box_width / 2),
        static_cast<int>(height - debug.first_point.y -
                         highlight_box_width / 2),
        highlight_box_width,
        highlight_box_width,
      };
      SDL_RenderDrawRect(renderer, &rect);
    }
    if (debug_options.chans_algorithm.show_intermediate_iteration >=
        debug.intermediate_subsets.size()) {
      debug_options.chans_algorithm.show_intermediate_iteration = -1;
    }
    if (debug_options.chans_algorithm.show_intermediate_iteration >= 0) {
      auto& subsets =
        debug.intermediate_subsets[debug_options.chans_algorithm
                                     .show_intermediate_iteration];
      auto& hulls = debug.intermediate_hulls[debug_options.chans_algorithm
                                               .show_intermediate_iteration];
      auto& found_point_iterations =
        debug.intermediate_found_hull_points[debug_options.chans_algorithm
                                               .show_intermediate_iteration];
      auto& best_found_point_iterations =
        debug.intermediate_found_points[debug_options.chans_algorithm
                                          .show_intermediate_iteration];
      if (debug_options.chans_algorithm.show_intermediate_subset >=
          subsets.size()) {
        debug_options.chans_algorithm.show_intermediate_subset = -1;
      }
      if (debug_options.chans_algorithm.show_intermediate_subset >= 0) {
        auto& subset =
          subsets[debug_options.chans_algorithm.show_intermediate_subset];
        auto& hull =
          hulls[debug_options.chans_algorithm.show_intermediate_subset];
        for (auto& p : subset) {
          SDL_SetRenderDrawColor(renderer, 0, 255, 255, SDL_ALPHA_OPAQUE);
          SDL_Rect rect{
            static_cast<int>(p.x - highlight_box_width / 2),
            static_cast<int>(height - p.y - highlight_box_width / 2),
            highlight_box_width,
            highlight_box_width,
          };
          SDL_RenderDrawRect(renderer, &rect);
        }
        draw(hull);
      }
      if (debug_options.chans_algorithm
            .show_intermediate_jarvis_found_points_iteration >= 0) {
        auto& points = found_point_iterations
          [debug_options.chans_algorithm
             .show_intermediate_jarvis_found_points_iteration];
        auto& best_point = best_found_point_iterations
          [debug_options.chans_algorithm
             .show_intermediate_jarvis_found_points_iteration];
        for (auto& p : points) {
          SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
          SDL_Rect rect{
            static_cast<int>(p.x - highlight_box_width / 2),
            static_cast<int>(height - p.y - highlight_box_width / 2),
            highlight_box_width,
            highlight_box_width,
          };
          SDL_RenderDrawRect(renderer, &rect);
        }
        SDL_SetRenderDrawColor(renderer, 255, 127, 127, SDL_ALPHA_OPAQUE);
        SDL_Rect rect{
          static_cast<int>(best_point.x - highlight_box_width / 2),
          static_cast<int>(height - best_point.y - highlight_box_width / 2),
          highlight_box_width,
          highlight_box_width,
        };
        SDL_RenderDrawRect(renderer, &rect);
      }
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
            solver.generate_points(
              wykobi::make_rectangle<double>(
                width * 0.25f, height * 0.75f, width * 0.75f, height * 0.25f),
              solver.num_points_to_generate);
            solver.solve();
            break;
          case SDL_SCANCODE_BACKSPACE:
            solver.generate_points(wykobi::make_rectangle<double>(0, 0, 0, 0),
                                   0);
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
    solver.generate_points(
      wykobi::make_rectangle<double>(
        width * 0.25f, height * 0.75f, width * 0.75f, height * 0.25f),
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
