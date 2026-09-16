/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
04.08.26, 18:33
*/

//! Components for standard icon sizes

use dioxus::prelude::*;
use dioxus_free_icons::{Icon, IconShape};

pub use dioxus_free_icons::icons::ld_icons as ld;

use crate::components::dynamic_icon::{DynIcon, DynIconType};

// supertrait that mandates all traits required for icon objects.
// This allows me to simplify components taking icons as props.
// TODO: would like to use trait aliases here in the future https://doc.rust-lang.org/beta/unstable-book/language-features/trait-alias.html
pub trait IconType: IconShape + Clone + PartialEq + 'static {}
// implement this for all icons.
impl<T: IconShape + Clone + PartialEq + 'static> IconType for T {}

/// Component Props for customized icon components
#[derive(PartialEq, Props, Clone)]
pub struct SizedIconProps<T: IconType> {
    /// The icon shape to use.
    pub icon: T,
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

#[component]
pub fn IconS<T: IconType>(props: SizedIconProps<T>) -> Element {
    rsx! {
        Icon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-s {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

#[component]
pub fn IconM<T: IconType>(props: SizedIconProps<T>) -> Element {
    rsx! {
        Icon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-m {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

#[component]
pub fn IconXL<T: IconType>(props: SizedIconProps<T>) -> Element {
    rsx! {
        Icon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-xl {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

/// This icon has no set size and no size class.
/// It is useful where size is changed dynamically 
/// in CSS (e.g. Navbar).
#[component]
pub fn IconSizeless<T: IconType>(props: SizedIconProps<T>) -> Element {
    rsx! {
        Icon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon {props.class}",
            title: props.title,
            style: props.style
        }
    }
}



// sized dynamic icons

/// Component Props for customized icon components
#[derive(PartialEq, Props, Clone)]
pub struct DynSizedIconProps {
    /// The icon shape to use.
    pub icon: DynIconType,
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

#[component]
pub fn DynIconS(props: DynSizedIconProps) -> Element {
    rsx! {
        DynIcon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-s {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

#[component]
pub fn DynIconM(props: DynSizedIconProps) -> Element {
    rsx! {
        DynIcon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-m {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

#[component]
pub fn DynIconXL(props: DynSizedIconProps) -> Element {
    rsx! {
        DynIcon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-xl {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

/// This icon has no set size and no size class.
/// It is useful where size is changed dynamically 
/// in CSS (e.g. Navbar).
#[component]
pub fn DynIconSizeless(props: DynSizedIconProps) -> Element {
    rsx! {
        DynIcon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon {props.class}",
            title: props.title,
            style: props.style
        }
    }
}