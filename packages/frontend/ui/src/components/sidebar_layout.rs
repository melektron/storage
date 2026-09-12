/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
31.08.26, 15:01
*/

//! This is a surrounding component used by most view components.
//! It is not used as a "router layout", instead being explicitly
//! used inside the view components in addition to the router layout

use std::rc::Rc;

use anyhow::anyhow;
use anyhow::Context;
use dioxus::prelude::*;
use dioxus_free_icons::icons::ld_icons as ld;
use dioxus_free_icons::IconShape;

use crate::components::actionbar_button::ActionbarButton;
use crate::components::dynamic_icon::DynIconType;
use crate::components::sized_icon::IconM;
//use web_sys::wasm_bindgen::JsCast;

const SIDEBAR_LAYOUT_CSS: Asset = asset!("./sidebar_layout.css");

pub enum Layout {
    Split,
    OnlyMain,
    OnlySidebar,
}


#[component]
pub fn SidebarLayout(
    #[props(into)]
    title: String,
    sidebar: Element,
    main_view: Element,
) -> Element {
    let mut dragging_pointer = use_signal(|| None::<i32>);
    let mut drag_offset = use_signal(|| 0 as f64);
    
    // TODO: Somehow persist these across sessions or at least navigations (local storage)
    let mut layout = use_signal(|| Layout::Split);
    let mut sidebar_width = use_signal(|| 200 as u32); // sidebar is 200px by default

    let layout_class = match *layout.read() {
        Layout::Split => "sbview-layout-split",
        Layout::OnlyMain => "sbview-layout-only-main",
        Layout::OnlySidebar => "sbview-layout-only-sidebar",
    };

    let layout_button_icon: DynIconType = match *layout.read() {
        Layout::Split => &ld::LdPanelLeftClose,
        Layout::OnlyMain => &ld::LdPanelRightClose,
        Layout::OnlySidebar => &ld::LdPanelLeft,
    };

    let a: &dyn IconShape = &ld::LdAArrowDown;
    let b = a.clone();

    // TODO: do we really need clone here? apparently singals iplement copy.
    let mut rotate_layout = move |_| {
        tracing::warn!("called rotate");
        let new_layout = match *layout.read() {
            Layout::Split => Layout::OnlyMain,
            Layout::OnlyMain => Layout::OnlySidebar,
            Layout::OnlySidebar => Layout::Split,
        };
        layout.set(new_layout);
    };

    rsx! {
        document::Link { rel: "stylesheet", href: SIDEBAR_LAYOUT_CSS }
        document::Title { "test" }

        div {
            class: "sbview-container {layout_class}",
            style: "--sidebar-width: {sidebar_width}px",
            
            // We capture move and up events on the the container instead of the
            // divider because we can't universally implement pointer capture.
            // This is an adequate (though not perfect) workaround. See below for details.
            onpointermove: move |evt| {
                if let Some(pointer_id) = *dragging_pointer.read() && pointer_id == evt.pointer_id() {
                    sidebar_width.set((evt.client_coordinates().x - *drag_offset.read()) as u32);
                }
            },
            onpointerup: move |evt| {
                let mut pointer_id = dragging_pointer.write();
                if pointer_id.is_none() { return }
                if let Some(id) = *pointer_id && id != evt.pointer_id() { return }
                *pointer_id = None;
            },

            div {
                class: "sbview-title-bar",
                ActionbarButton {
                    text: "hi",
                    icon: layout_button_icon,
                    onclick: move |evt| rotate_layout(evt),
                }
                ActionbarButton {
                    text: "hi",
                    icon: &ld::LdPanelLeftClose,
                    onclick: move |evt| rotate_layout(evt),
                }
                ActionbarButton {
                    text: "hi",
                    icon: &ld::LdPanelLeftDashed,
                    onclick: move |evt| rotate_layout(evt),
                }
                ActionbarButton {
                    text: "hi",
                    icon: &ld::LdPanelLeftOpen,
                    onclick: move |evt| rotate_layout(evt),
                }
            }

            div {
                class: "sbview-sidebar sbview-scrollable-y",
                nav {
                    class: "sbview-sidebar-inner",
                    { sidebar }
                }
            }

            div {
                class: "sbview-divider",
                // save element reference for pointer capturing
                onpointerdown: move |evt| {
                    // Unfortunately we can't properly implement pointer capture
                    // in a fullstack app... at least not in a platform agnostic way.
                    // That's because this code doesn't compile on the server or other 
                    // native targets.
                    // TODO: get this working with some feature flags or something...
                    //let mut inner = move || -> anyhow::Result<()> {
                    //    let pointer_event = evt
                    //        .downcast::<web_sys::PointerEvent>()
                    //        .ok_or(anyhow!("cast failed"))?;
                    //
                    //    let target = pointer_event
                    //        .current_target()
                    //        .ok_or(anyhow!("no target"))?
                    //        .dyn_into::<web_sys::Element>()
                    //        .map_err(|_| anyhow!("element cast failed"))?;
                    //
                    //    target
                    //        .set_pointer_capture(pointer_event.pointer_id())
                    //        .map_err(|js_err| anyhow!("pointer capture failed: {js_err:?}"))?;
                    //
                    //    dragging.set(true);
                    //
                    //    tracing::info!("dragstart {}", pointer_event.pointer_id());
                    //    Ok(())
                    //};
                    //
                    //if let Err(err) = inner() {
                    //    
                    //};
                    dragging_pointer.set(Some(evt.pointer_id()));
                    // calculate the offset to get new panel width from client coords
                    drag_offset.set(evt.client_coordinates().x - (*sidebar_width.read() as f64));
                },
            }

            div {
                class: "sbview-main-container sbview-scrollable-y",
                main {
                    class: "sbview-main-inner",
                    { main_view }
                }
            }
        }

    }
}