#include <list>

#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>
#include <SDL.h>
#include <SDL_render.h>
#include <imgui.h>
#include <imgui_sdl.h>
#include <imgui_impl_sdl.h>

#if __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace algorithm {
using namespace wykobi;
using namespace wykobi::algorithm;

template<typename T>
struct convex_hull_quick_hull;

template<typename T>
struct convex_hull_quick_hull<point2d<T>>
{
public:
  struct debug_data_t
  {
    segment2d first_horizontal_split;
    std::vector<triangle2d> split_triangles;
  };
  template<typename InputIterator, typename OutputIterator>
  convex_hull_quick_hull(InputIterator begin,
                         InputIterator end,
                         OutputIterator out,
                         debug_data_t& debug)
  {
    // If the input is a point, line segment or triangle, then there is no work
    // to do. They will always form a convex set.
    if (std::distance(begin, end) <= 3) {
      std::copy(begin, end, out);
      return;
    }

    std::list<point2d<T>> point_list;

    // Find leftmost and rightmost points and add to convex hull
    const auto [leftmost, rightmost] = std::minmax_element(
      begin, end, [](const point2d<T>& a, const point2d<T>& b) -> bool {
        return a.x < b.x;
      });
    debug.first_horizontal_split = make_segment(*leftmost, *rightmost);
    auto leftmost_it = point_list.insert(
      point_list.end(), make_point<T>(leftmost->x, leftmost->y));
    auto rightmost_it = point_list.insert(
      point_list.end(), make_point<T>(rightmost->x, rightmost->y));

    // Segment divides the remaining (n − 2) points into two groups S1 and S2
    // where S1 are points in the set which are on the right side of the
    // oriented line from leftmost to rightmost
    // and S2 are points in the set which are on the right side of the oriented
    // line from rightmost to leftmost
    std::vector<typename std::iterator_traits<InputIterator>::value_type> s1;
    std::vector<typename std::iterator_traits<InputIterator>::value_type> s2;
    s1.reserve(end - begin - 1);
    s2.reserve(end - begin - 1);
    for (InputIterator it = begin; it != end; ++it) {
      if (it == leftmost || it == rightmost) {
        continue;
      }
      switch (orientation(*leftmost, *rightmost, *it)) {
        case RightHandSide:
          s1.push_back(wykobi::make_point(it->x, it->y));
          break;
        case CollinearOrientation:
        case LeftHandSide:
          s2.push_back(wykobi::make_point(it->x, it->y));
          break;
      }
    }

    // insert with std::next(leftmost_it) which is rightmost_it so that it is
    // inserted after leftmost and before rightmost
    find_hull(s1.cbegin(),
              s1.cend(),
              leftmost,
              rightmost,
              point_list,
              std::next(leftmost_it),
              debug);
    // insert with std::next(leftmost_it) which is point_list.end so that it is
    // inserted after rightmost, which in a loop is before leftmost
    find_hull(s2.cbegin(),
              s2.cend(),
              rightmost,
              leftmost,
              point_list,
              std::next(rightmost_it),
              debug);

    for (const auto& point : point_list) {
      (*out++) = make_point<T>(point.x, point.y);
    }
  }

private:
  inline T distance2(const T& x1, const T& y1, const T& x2, const T& y2)
  {
    const T dx = (x1 - x2);
    const T dy = (y1 - y2);

    return dx * dx + dy * dy;
  }

  inline T minimum_distance2_from_point_to_segment(const T& px,
                                                   const T& py,
                                                   const T& x1,
                                                   const T& y1,
                                                   const T& x2,
                                                   const T& y2)
  {
    T nx = T(0.0);
    T ny = T(0.0);

    closest_point_on_segment_from_point(x1, y1, x2, y2, px, py, nx, ny);

    return distance2(px, py, nx, ny);
  }

  inline T minimum_distance2_from_point_to_segment(const point2d<T>& point,
                                                   const segment<T, 2>& segment)
  {
    return minimum_distance2_from_point_to_segment(
      point.x, point.y, segment[0].x, segment[0].y, segment[1].x, segment[1].y);
  }

  /// Find points on convex hull from the set Sk of points
  /// that are on the right side of the oriented line from P to Q
  template<typename InputIterator, typename OutputIterator>
  void find_hull(InputIterator begin,
                 InputIterator end,
                 InputIterator segment_0,
                 InputIterator segment_1,
                 std::list<point2d<T>> point_list,
                 OutputIterator segment_it,
                 debug_data_t& debug)
  {
    // Sk has no points
    if (begin == end) {
      return;
    }
    // Find the farthest point from segment
    auto farthest = end;
    {
      T greatest_distance2 = T(0);
      for (auto it = begin; it != end; ++it) {
        auto d2 = minimum_distance2_from_point_to_segment(
          *it, make_segment(*segment_0, *segment_1));
        if (d2 > greatest_distance2) {
          greatest_distance2 = d2;
          farthest = it;
        }
      }
    }
    if (farthest == end) {
      return;
    }

    // Add point C to convex hull at the location between P and Q
    auto farthest_it =
      point_list.insert(segment_it, make_point<T>(farthest->x, farthest->y));
    // Three points P, Q, and C partition the remaining points of Sk into 3
    // subsets: S0, S1, and S2 where S0 are points inside triangle PCQ, S1 are
    // points on the right side of the oriented line from P to C, and S2 are
    // points on the right side of the oriented line from C to Q.
    std::vector<typename std::iterator_traits<InputIterator>::value_type> s1;
    std::vector<typename std::iterator_traits<InputIterator>::value_type> s2;
    s1.reserve(end - begin - 1);
    s2.reserve(end - begin - 1);
    for (InputIterator it = begin; it != end; ++it) {
      if (it == segment_0 || it == segment_1 || it == farthest) {
        continue;
      }
      if (orientation(*segment_0, *farthest, *it) == RightHandSide) {
        s1.push_back(wykobi::make_point(it->x, it->y));
      } else if (orientation(*farthest, *segment_1, *it) == RightHandSide) {
        s2.push_back(wykobi::make_point(it->x, it->y));
      }
    }
    debug.split_triangles.push_back(
      make_triangle(*segment_0, *farthest, *segment_1));
    find_hull(s1.cbegin(),
              s1.cend(),
              segment_0,
              farthest,
              point_list,
              farthest_it,
              debug);
    find_hull(s2.cbegin(),
              s2.cend(),
              farthest,
              segment_1,
              point_list,
              std::next(farthest_it),
              debug);
  }
};
} // namespace algorithm

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
