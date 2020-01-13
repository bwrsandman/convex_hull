#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>

#include <SDL.h>
#include <SDL_render.h>

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
  template<typename InputIterator, typename OutputIterator>
  convex_hull_quick_hull(InputIterator begin,
                         InputIterator end,
                         OutputIterator out)
  {
    if (std::distance(begin, end) <= 3) {
      std::copy(begin, end, out);
      return;
    }
  }
};
} // namespace algorithm

template<class T>
class Solver
{
public:
  Solver() = default;
  ~Solver() = default;

  void add_point(T x, T y)
  {
    auto& p = point_list.emplace_back();
    p.x = x;
    p.y = y;
  }

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
      point_list.begin(), point_list.end(), std::back_inserter(convex_hull));
  }

  template<typename InputIterator, typename OutputIterator>
  void solve_quick_hull(InputIterator begin,
                        InputIterator end,
                        OutputIterator out)
  {
    convex_hull.clear();
    algorithm::convex_hull_quick_hull<wykobi::point2d<T>>(begin, end, out);
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

private:
  std::vector<wykobi::point2d<T>> point_list;
  wykobi::polygon<T, 2> convex_hull;

  friend class Renderer;
};

class Renderer
{
public:
  Renderer()
    : window(nullptr)
    , renderer(nullptr)
  {}
  ~Renderer() { terminate(); }

  bool initialize()
  {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
      return false;
    }

    window = SDL_CreateWindow(
      "quickhull", SDL_HINT_DEFAULT, SDL_HINT_DEFAULT, 640, 480, 0);
    if (!window) {
      terminate();
      return false;
    }

    renderer = SDL_CreateRenderer(window, 0, 0);
    if (!renderer) {
      terminate();
      return false;
    }
    return true;
  }

  void terminate() noexcept
  {
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
  void draw(const Solver<T>& solver)
  {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    draw(solver.point_list);
    draw(solver.convex_hull);

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
                         polygon[i].y,
                         polygon[i + 1].x,
                         polygon[i + 1].y);
    }
    if (polygon.size() > 1) {
      SDL_RenderDrawLine(renderer,
                         polygon[polygon.size() - 1].x,
                         polygon[polygon.size() - 1].y,
                         polygon[0].x,
                         polygon[0].y);
    } else if (polygon.size() == 1) {
      SDL_RenderDrawPoint(renderer, polygon[0].x, polygon[0].y);
    }
  }

  template<class T>
  void draw(const std::vector<wykobi::point2d<T>>& point_list)
  {
    for (const auto& point : point_list) {
      SDL_RenderDrawPoint(renderer, point.x, point.y);
    }
  }

  void resolution(uint32_t& width, uint32_t& height) const
  {
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    width = w;
    height = h;
  }

private:
  SDL_Window* window;
  SDL_Renderer* renderer;
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
      if (event.type == SDL_QUIT) {
        return false;
      } else if (event.type == SDL_MOUSEBUTTONDOWN) {
        solver.add_point(event.button.x, event.button.y);
        solver.solve();
      } else if (event.type == SDL_KEYDOWN) {
#if __EMSCRIPTEN__
        switch (event.key.keysym.sym) { // Weird emscripten bug
#else
        switch (event.key.keysym.scancode) {
#endif
          case SDL_SCANCODE_SPACE:
            uint32_t width, height;
            renderer.resolution(width, height);
            solver.generate_points(width * 0.25f,
                                   height * 0.25f,
                                   width * 0.5f,
                                   height * 0.5f,
                                   num_points);
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
    solver.generate_points(
      width * 0.25f, height * 0.25f, width * 0.5f, height * 0.5f, num_points);
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
  static constexpr std::size_t num_points = 10;
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
