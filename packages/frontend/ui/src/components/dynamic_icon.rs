/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
12.09.26, 18:35
*/

//! Icon component for dioxus_free_icons that uses
//! dynamic dispatch -> this allows passing different
//! icons to components dynamically


use std::any::TypeId;

use dioxus::prelude::*;
use dioxus_free_icons::IconShape;

/// IconShape extension that allows comparison between
/// unit structs implementing IconShape (all the icon
/// definitions of dioxus_free_icons AFAIK)
pub trait DynIconShape: IconShape + 'static {
    fn type_id(&self) -> TypeId {
        TypeId::of::<Self>()
    }
}
// All unit structs (static) implementing IconShape should also have this.
impl<T> DynIconShape for T where T: IconShape + 'static {}
// Compare IconShape trait objects based on their struct type
impl PartialEq for &'static dyn DynIconShape {
    fn eq(&self, other: &Self) -> bool {
        self.type_id() == other.type_id()
    }
}

/// type alias for convenience
pub type DynIconType = &'static dyn DynIconShape;

/// Icon component Props
#[derive(PartialEq, Props, Clone)]
pub struct DynIconProps {
    /// The icon shape to use.
    pub icon: DynIconType,
    /// The height of the `<svg>` element. Defaults to 20. Pass None to omit.
    #[props(default = Some(20))]
    pub height: Option<u32>,
    /// The width of the `<svg>` element. Defaults to 20. Pass None to omit.
    #[props(default = Some(20))]
    pub width: Option<u32>,
    /// The color to use for filling the icon. Defaults to "currentColor".
    #[props(default = "currentColor".to_string())]
    pub fill: String,
    /// An class for the `<svg>` element.
    #[props(default = "".to_string())]
    pub class: String,
    /// An accessible, short-text description for the icon.
    pub title: Option<String>,
    /// The style of the `<svg>` element.
    pub style: Option<String>,
}

/// DynamicIcon component which generates SVG elements
#[allow(non_snake_case)]
pub fn DynIcon(props: DynIconProps) -> Element {
    let (fill, stroke, stroke_width) = props.icon.fill_and_stroke(&props.fill);
    rsx!(
        svg {
            class: "{props.class}",
            style: props.style,
            height: props.height.map(|height| height.to_string()),
            width: props.width.map(|width| width.to_string()),
            view_box: "{props.icon.view_box()}",
            xmlns: "{props.icon.xmlns()}",
            fill,
            stroke,
            stroke_width,
            stroke_linecap: "{props.icon.stroke_linecap()}",
            stroke_linejoin: "{props.icon.stroke_linejoin()}",
            if let Some(title_text) = props.title {
                title { "{title_text}" }
            }
            {props.icon.child_elements()}
        }
    )
}

/*
Questions:
- Does TypeId comparison always
- Obviously breaking change in usage, maybe provide in addition?
- Performance? Probably slower and less optimizable than static Icon...
- Any caveats that I haven't thought of?
*/