use crate::screen_geom::ScreenRectangle;
use crate::{
    hotkey, Color, EventCtx, EventLoopMode, GfxCtx, Key, ModalMenu, MultiKey, ScreenPt, Text,
    Warper,
};
use geom::{Distance, Polygon, Pt2D};

// Pixels
const BAR_WIDTH: f64 = 300.0;
const BAR_HEIGHT: f64 = 100.0;
const SLIDER_WIDTH: f64 = 50.0;
const SLIDER_HEIGHT: f64 = 120.0;

const HORIZ_PADDING: f64 = 60.0;
const VERT_PADDING: f64 = 20.0;

pub struct Slider {
    top_left: ScreenPt,
    current_percent: f64,
    mouse_on_slider: bool,
    dragging: bool,
}

impl Slider {
    // TODO Easier placement options.
    pub fn new(top_left_at: Option<ScreenPt>) -> Slider {
        Slider {
            top_left: top_left_at.unwrap_or_else(|| ScreenPt::new(0.0, 0.0)),
            current_percent: 0.0,
            mouse_on_slider: false,
            dragging: false,
        }
    }

    pub fn get_percent(&self) -> f64 {
        self.current_percent
    }

    pub fn get_value(&self, num_items: usize) -> usize {
        (self.current_percent * (num_items as f64 - 1.0)) as usize
    }

    pub fn set_percent(&mut self, ctx: &mut EventCtx, percent: f64) {
        assert!(percent >= 0.0 && percent <= 1.0);
        self.current_percent = percent;
        // Just reset dragging, to prevent chaos
        self.dragging = false;
        let pt = ctx.canvas.get_cursor_in_screen_space();
        self.mouse_on_slider = self.slider_geom().contains_pt(Pt2D::new(pt.x, pt.y));
    }

    pub fn set_value(&mut self, ctx: &mut EventCtx, idx: usize, num_items: usize) {
        self.set_percent(ctx, (idx as f64) / (num_items as f64 - 1.0));
    }

    // Returns true if the percentage changed.
    pub fn event(&mut self, ctx: &mut EventCtx) -> bool {
        if self.dragging {
            if ctx.input.get_moved_mouse().is_some() {
                let percent =
                    (ctx.canvas.get_cursor_in_screen_space().x - HORIZ_PADDING - self.top_left.x)
                        / BAR_WIDTH;
                self.current_percent = percent.min(1.0).max(0.0);
                return true;
            }
            if ctx.input.left_mouse_button_released() {
                self.dragging = false;
            }
        } else {
            if ctx.redo_mouseover() {
                let pt = ctx.canvas.get_cursor_in_screen_space();
                self.mouse_on_slider = self.slider_geom().contains_pt(Pt2D::new(pt.x, pt.y));
            }
            if ctx.input.left_mouse_button_pressed() {
                if self.mouse_on_slider {
                    self.dragging = true;
                } else {
                    // Did we click somewhere else on the bar?
                    let pt = ctx.canvas.get_cursor_in_screen_space();
                    if Polygon::rectangle_topleft(
                        Pt2D::new(
                            HORIZ_PADDING + self.top_left.x,
                            VERT_PADDING + self.top_left.y,
                        ),
                        Distance::meters(BAR_WIDTH),
                        Distance::meters(BAR_HEIGHT),
                    )
                    .contains_pt(Pt2D::new(pt.x, pt.y))
                    {
                        let percent = (pt.x - HORIZ_PADDING - self.top_left.x) / BAR_WIDTH;
                        self.current_percent = percent.min(1.0).max(0.0);
                        self.mouse_on_slider = true;
                        self.dragging = true;
                        return true;
                    }
                }
            }
        }
        false
    }

    pub fn draw(&self, g: &mut GfxCtx) {
        g.fork_screenspace();

        // A nice background for the entire thing
        g.draw_polygon(
            Color::grey(0.3),
            &Polygon::rectangle_topleft(
                Pt2D::new(self.top_left.x, self.top_left.y),
                Distance::meters(BAR_WIDTH + 2.0 * HORIZ_PADDING),
                Distance::meters(BAR_HEIGHT + 2.0 * VERT_PADDING),
            ),
        );
        g.canvas.mark_covered_area(ScreenRectangle {
            x1: self.top_left.x,
            y1: self.top_left.y,
            x2: self.top_left.x + BAR_WIDTH + 2.0 * HORIZ_PADDING,
            y2: self.top_left.y + BAR_HEIGHT + 2.0 * VERT_PADDING,
        });

        // The bar
        g.draw_polygon(
            Color::WHITE,
            &Polygon::rectangle_topleft(
                Pt2D::new(
                    self.top_left.x + HORIZ_PADDING,
                    self.top_left.y + VERT_PADDING,
                ),
                Distance::meters(BAR_WIDTH),
                Distance::meters(BAR_HEIGHT),
            ),
        );

        // Show the progress
        if self.current_percent != 0.0 {
            g.draw_polygon(
                Color::GREEN,
                &Polygon::rectangle_topleft(
                    Pt2D::new(
                        self.top_left.x + HORIZ_PADDING,
                        self.top_left.y + VERT_PADDING,
                    ),
                    Distance::meters(self.current_percent * BAR_WIDTH),
                    Distance::meters(BAR_HEIGHT),
                ),
            );
        }

        // The actual slider
        g.draw_polygon(
            if self.mouse_on_slider {
                Color::YELLOW
            } else {
                Color::grey(0.7)
            },
            &self.slider_geom(),
        );
    }

    fn slider_geom(&self) -> Polygon {
        Polygon::rectangle_topleft(
            Pt2D::new(
                self.top_left.x + HORIZ_PADDING + self.current_percent * BAR_WIDTH
                    - (SLIDER_WIDTH / 2.0),
                self.top_left.y + VERT_PADDING - (SLIDER_HEIGHT - BAR_HEIGHT) / 2.0,
            ),
            Distance::meters(SLIDER_WIDTH),
            Distance::meters(SLIDER_HEIGHT),
        )
    }
}

pub struct ItemSlider<T> {
    items: Vec<T>,
    slider: Slider,
    menu: ModalMenu,

    prev: String,
    next: String,
    first: String,
    last: String,
}

impl<T> ItemSlider<T> {
    pub fn new(
        items: Vec<T>,
        menu_title: &str,
        noun: &str,
        other_choices: Vec<(Option<MultiKey>, &str)>,
        ctx: &mut EventCtx,
    ) -> ItemSlider<T> {
        // Lifetime funniness...
        let mut choices = other_choices.clone();

        let prev = format!("previous {}", noun);
        let next = format!("next {}", noun);
        let first = format!("first {}", noun);
        let last = format!("last {}", noun);
        choices.extend(vec![
            (hotkey(Key::LeftArrow), prev.as_str()),
            (hotkey(Key::RightArrow), next.as_str()),
            (hotkey(Key::Comma), first.as_str()),
            (hotkey(Key::Dot), last.as_str()),
        ]);

        ItemSlider {
            items,
            slider: Slider::new(None),
            menu: ModalMenu::new(menu_title, choices, ctx),

            prev,
            next,
            first,
            last,
        }
    }

    // Returns true if the value changed.
    pub fn event(&mut self, ctx: &mut EventCtx, menu_prompt: Option<Text>) -> bool {
        let current = self.slider.get_value(self.items.len());

        self.menu.handle_event(ctx, menu_prompt);

        if current != self.items.len() - 1 && self.menu.action(&self.next) {
            self.slider.set_value(ctx, current + 1, self.items.len());
        } else if current != self.items.len() - 1 && self.menu.action(&self.last) {
            self.slider.set_percent(ctx, 1.0);
        } else if current != 0 && self.menu.action(&self.prev) {
            self.slider.set_value(ctx, current - 1, self.items.len());
        } else if current != 0 && self.menu.action(&self.first) {
            self.slider.set_percent(ctx, 0.0);
        }

        self.slider.event(ctx);

        self.slider.get_value(self.items.len()) != current
    }

    pub fn draw(&self, g: &mut GfxCtx) {
        self.menu.draw(g);
        self.slider.draw(g);
    }

    pub fn get(&self) -> (usize, &T) {
        let idx = self.slider.get_value(self.items.len());
        (idx, &self.items[idx])
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn action(&mut self, name: &str) -> bool {
        self.menu.action(name)
    }

    // TODO Consume self
    pub fn consume_all_items(&mut self) -> Vec<T> {
        std::mem::replace(&mut self.items, Vec::new())
    }
}

pub struct WarpingItemSlider<T> {
    slider: ItemSlider<(Pt2D, T)>,
    warper: Option<Warper>,
}

impl<T> WarpingItemSlider<T> {
    // Note other_choices is hardcoded to quitting.
    pub fn new(
        items: Vec<(Pt2D, T)>,
        menu_title: &str,
        noun: &str,
        ctx: &mut EventCtx,
    ) -> WarpingItemSlider<T> {
        WarpingItemSlider {
            warper: Some(Warper::new(ctx, items[0].0)),
            slider: ItemSlider::new(
                items,
                menu_title,
                noun,
                vec![(hotkey(Key::Escape), "quit")],
                ctx,
            ),
        }
    }

    // Done when None. If the bool is true, done warping.
    pub fn event(
        &mut self,
        ctx: &mut EventCtx,
        menu_prompt: Option<Text>,
    ) -> Option<(EventLoopMode, bool)> {
        // Don't block while we're warping
        let (ev_mode, done_warping) = if let Some(ref warper) = self.warper {
            if let Some(mode) = warper.event(ctx) {
                (mode, false)
            } else {
                self.warper = None;
                (EventLoopMode::InputOnly, true)
            }
        } else {
            (EventLoopMode::InputOnly, false)
        };

        let changed = self.slider.event(ctx, menu_prompt);

        if self.slider.action("quit") {
            return None;
        } else if !changed {
            return Some((ev_mode, done_warping));
        }

        let (_, (pt, _)) = self.slider.get();
        self.warper = Some(Warper::new(ctx, *pt));
        // We just created a new warper, so...
        Some((EventLoopMode::Animation, done_warping))
    }

    pub fn draw(&self, g: &mut GfxCtx) {
        self.slider.draw(g);
    }

    pub fn get(&self) -> (usize, &T) {
        let (idx, (_, data)) = self.slider.get();
        (idx, data)
    }

    pub fn len(&self) -> usize {
        self.slider.len()
    }
}
