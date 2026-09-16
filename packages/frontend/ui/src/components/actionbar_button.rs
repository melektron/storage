/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
08.09.26, 14:27
*/

//! square button with icon to be used in various action bars (e.g. sbview)

use dioxus::prelude::*;

pub use dioxus_free_icons::icons::ld_icons as ld;

use crate::components::{dynamic_icon::{DynIcon, DynIconType}, sized_icon::{DynIconS, DynIconSizeless, IconM, IconS, IconType}};


const ACTIONBAR_BUTTON_CSS: Asset = asset!("./actionbar_button.css");

#[component]
pub fn ActionbarButton(
    text: String,
    #[props(into, default = "")]
    class: String,
    icon: DynIconType,
    onclick: Option<EventHandler<MouseEvent>>
) -> Element {
    rsx! {
        document::Link { rel: "stylesheet", href: ACTIONBAR_BUTTON_CSS }
        button {
            class: "actionbar-button {class}",
            // passing on optional event handlers is unfortunately not possible yet
            // https://discord.com/channels/8998coincitentally51952891002890/943190605067079712/1350519606921531402
            // https://github.com/DioxusLabs/dioxus/issues/1005#issuecomment-1542310050
            onclick: move |evt| if let Some(onclick) = onclick { onclick.call(evt) },
            DynIconSizeless {
                icon: icon
            } 
        }
    }
}