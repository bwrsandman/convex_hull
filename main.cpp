#include <list>

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
        case LeftHandSide:
          s2.push_back(wykobi::make_point(it->x, it->y));
          break;
        case CollinearOrientation:
          // TODO: Add to one of the two sets or ignore?
          assert(false);
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
              std::next(leftmost_it));
    // insert with std::next(leftmost_it) which is point_list.end so that it is
    // inserted after rightmost, which in a loop is before leftmost
    find_hull(s2.cbegin(),
              s2.cend(),
              rightmost,
              leftmost,
              point_list,
              std::next(rightmost_it));

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
                 OutputIterator segment_it)
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
    assert(farthest != end);

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
    find_hull(
      s1.cbegin(), s1.cend(), segment_0, farthest, point_list, segment_it);
    find_hull(s2.cbegin(),
              s2.cend(),
              farthest,
              segment_1,
              point_list,
              std::next(farthest_it));
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

    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    width = w;
    height = h;

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
                         height - polygon[i].y,
                         polygon[i + 1].x,
                         height - polygon[i + 1].y);
    }
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    if (polygon.size() > 1) {
      SDL_RenderDrawLine(renderer,
                         polygon[polygon.size() - 1].x,
                         height - polygon[polygon.size() - 1].y,
                         polygon[0].x,
                         height - polygon[0].y);
    } else if (polygon.size() == 1) {
      SDL_RenderDrawPoint(renderer, polygon[0].x, height - polygon[0].y);
    }

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

  template<class T>
  void draw(const std::vector<wykobi::point2d<T>>& point_list)
  {
    for (const auto& point : point_list) {
      SDL_RenderDrawPoint(renderer, point.x, height - point.y);
    }
  }

  void resolution(uint32_t& out_width, uint32_t& out_height) const
  {
    out_width = width;
    out_height = height;
  }

private:
  SDL_Window* window;
  SDL_Renderer* renderer;
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
      uint32_t width, height;
      renderer.resolution(width, height);
      if (event.type == SDL_QUIT) {
        return false;
      } else if (event.type == SDL_MOUSEBUTTONDOWN) {
        solver.add_point(event.button.x, height - event.button.y);
        solver.solve();
      } else if (event.type == SDL_KEYDOWN) {
#if __EMSCRIPTEN__
        switch (event.key.keysym.sym) { // Weird emscripten bug
#else
        switch (event.key.keysym.scancode) {
#endif
          case SDL_SCANCODE_SPACE:
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
