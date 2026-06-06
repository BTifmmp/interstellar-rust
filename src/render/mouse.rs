use ::macroquad::prelude::*;

pub fn update_mouse_lock() -> Option<bool> {
    if is_key_pressed(KeyCode::Escape) {
        set_cursor_grab(false);
        show_mouse(true);
        return Some(false);
    }

    if is_mouse_button_pressed(MouseButton::Left) {
        set_cursor_grab(true);
        show_mouse(false);
        return Some(true);
    }

    return None;
}
