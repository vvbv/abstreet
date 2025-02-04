mod canvas;
mod color;
mod drawing;
mod event;
mod event_ctx;
mod input;
mod runner;
mod screen_geom;
mod text;
mod widgets;

pub use crate::canvas::{Canvas, HorizontalAlignment, VerticalAlignment, BOTTOM_LEFT, CENTERED};
pub use crate::color::Color;
pub use crate::drawing::{GeomBatch, GfxCtx};
pub use crate::event::{hotkey, lctrl, Event, Key, MultiKey};
pub use crate::event_ctx::{Drawable, EventCtx, Prerender};
pub use crate::input::UserInput;
pub use crate::runner::{run, EventLoopMode, GUI};
pub use crate::screen_geom::ScreenPt;
pub use crate::text::{Text, HOTKEY_COLOR};
pub use crate::widgets::{
    Autocomplete, ItemSlider, LogScroller, ModalMenu, ScrollingMenu, Slider, TextBox, Warper,
    WarpingItemSlider, Wizard, WrappedWizard,
};

pub enum InputResult<T: Clone> {
    Canceled,
    StillActive,
    Done(String, T),
}
