/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
08.09.26, 14:27
*/

//! UI components for use in action bars

use dioxus::prelude::*;

pub use dioxus_free_icons::icons::ld_icons as ld;

use crate::components::{dynamic_icon::{DynIcon, DynIconType}, sized_icon::{DynIconS, DynIconSizeless, IconM, IconS, IconType}};


const ACTIONBAR_CSS: Asset = asset!("./actionbar.css");

/// square button with icon to be used in various action bars (e.g. sbview)
#[component]
pub fn ActionbarButton(
    tooltip: String,
    #[props(into, default = "")]
    class: String,
    icon: DynIconType,
    onclick: Option<EventHandler<MouseEvent>>,
    #[props(extends = GlobalAttributes)]
    attributes: Vec<Attribute>,
) -> Element {
    rsx! {
        document::Link { rel: "stylesheet", href: ACTIONBAR_CSS }
        button {
            class: "actionbar-button {class}",
            title: tooltip,
            // passing on optional event handlers is unfortunately not possible yet
            // https://discord.com/channels/8998coincitentally51952891002890/943190605067079712/1350519606921531402
            // https://github.com/DioxusLabs/dioxus/issues/1005#issuecomment-1542310050
            onclick: move |evt| if let Some(onclick) = onclick { onclick.call(evt) },
            ..attributes,
            DynIconSizeless {
                icon: icon
            } 
        }
    }
}

#[derive(Clone, PartialEq)]
pub struct ActionbarSelectorOption<T> {
    pub value: T,
    pub icon: DynIconType,
    pub tooltip: String,
    pub class: String,
}

impl<T> ActionbarSelectorOption<T> {
    pub fn new(value: T, icon: DynIconType, tooltip: &str, class: &str) -> Self {
        ActionbarSelectorOption { value, icon, tooltip: tooltip.to_owned(), class: class.to_owned() }
    }
}


/// array of ActionbarButton-like items where one can be selected
#[component]
pub fn ActionbarSelector<T, I>(
    options: I,
    initial: Option<T>,
    onchange: Option<EventHandler<T>>,
    #[props(into, default = "")]
    class: String,
    #[props(extends = GlobalAttributes)]
    attributes: Vec<Attribute>,
) -> Element 
where 
    T: Clone + PartialEq + 'static,
    I: Clone + PartialEq + IntoIterator<Item = ActionbarSelectorOption<T>> + 'static
{
    let mut selected_index = use_signal(el_std::clone!(options => || {
        // compute the initial index for the initial value if provided 
        initial.map_or(
            None, 
            |initial_idx| options
                .into_iter()
                .enumerate()
                .find_map(
                    |(idx, option)| if option.value == initial_idx {
                        Some(idx)
                    } else { 
                        None 
                    }
                )
        )
    }));

    rsx! {
        document::Link { rel: "stylesheet", href: ACTIONBAR_CSS }
        div {
            class: "actionbar-selector {class}",
            ..attributes,
            for (index, option) in options.into_iter().enumerate() {
                button {
                    // TODO: in the future maybe specify active by value instead of index,
                    // but for now we don't intend this to be used with changing options
                    key: "{index}",
                    class: "actionbar-selector-button {option.class}",
                    class: if Some(index) == *selected_index.read() { "actionbar-selector-button-selected" },
                    title: option.tooltip,
                    onclick: move |_| {
                        selected_index.set(Some(index));
                        if let Some(handler) = onchange {
                            handler.call(option.value.clone());
                        }
                    },
                    DynIconSizeless {
                        icon: option.icon
                    } 
                }
            }
        }
        
    }
}