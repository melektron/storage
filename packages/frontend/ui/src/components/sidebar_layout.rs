/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
31.08.26, 15:01
*/

//! This is a surrounding component used by most view components.
//! It is not used as a "router layout", instead being explicitly
//! used inside the view components in addition to the router layout

use anyhow::anyhow;
use dioxus::prelude::*;
use dioxus_free_icons::icons::ld_icons as ld;

use crate::components::actionbar_button::ActionbarButton;
use crate::components::dynamic_icon::DynIconType;

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
    let sidebar_min_width = 200u32;
    let main_min_width = 200u32;
    let mut sidebar_width = use_signal(|| 200 as u32); // sidebar is 200px by default

    let layout_attr = match *layout.read() {
        Layout::Split => "split",
        Layout::OnlyMain => "only-main",
        Layout::OnlySidebar => "only-sidebar",
    };

    let layout_button_icon: DynIconType = match *layout.read() {
        Layout::Split => &ld::LdPanelLeftClose,
        Layout::OnlyMain => &ld::LdPanelRightClose,
        Layout::OnlySidebar => &ld::LdPanelLeft,
    };

    let mut rotate_layout = move |_| {
        tracing::warn!("called rotate");
        let new_layout = match *layout.read() {
            Layout::Split => Layout::OnlyMain,
            Layout::OnlyMain => Layout::OnlySidebar,
            Layout::OnlySidebar => Layout::Split,
        };
        layout.set(new_layout);
    };

    let handle_resize_move = move |evt: Event<PointerData>| {
        // prevent accidental text selection while dragging
        evt.prevent_default();

        if let Some(pointer_id) = *dragging_pointer.read() && pointer_id == evt.pointer_id() {
            let target_pos = (evt.client_coordinates().x - *drag_offset.read()) as u32;
            sidebar_width.set(std::cmp::max(target_pos, sidebar_min_width));
        }
    };
    let handle_resize_stop = move |evt: Event<PointerData>| {
        let mut pointer_id = dragging_pointer.write();
        if pointer_id.is_none() { return }
        if let Some(id) = *pointer_id && id != evt.pointer_id() { return }
        *pointer_id = None;
    };

    rsx! {
        document::Link { rel: "stylesheet", href: SIDEBAR_LAYOUT_CSS }
        document::Title { "test" }

        div {
            class: "sbview",
            class: if let Some(_) = *dragging_pointer.read() { "mod-is-resizing" },
            style: "--sidebar-width: {sidebar_width}px; --sidebar-min-width: {sidebar_min_width}px; --main-min-width: {main_min_width}px;",
            "data-layout": layout_attr,
            
            // On native platforms, we capture move and up events on the the 
            // container instead of the divider because we can't implement pointer capture.
            // It seems that some browsers sometimes quasi-capture the pointer to the viewport, and 
            // since the sbview takes up pretty much the entire viewport this is an adequate 
            // (though not perfect) workaround to pointer capture.
            // Does have the disadvantage of firing many unnecessary events while not resizing.
            onpointermove: handle_resize_move,
            onpointerup: handle_resize_stop,

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
                    onclick: move |_| {
                        let new_layout = match *layout.read() {
                            Layout::Split => Layout::OnlyMain,
                            Layout::OnlyMain => Layout::Split,
                            Layout::OnlySidebar => Layout::Split,
                        };
                        layout.set(new_layout);
                    }
                }
                ActionbarButton {
                    text: "hi",
                    icon: &ld::LdPanelLeftDashed,
                    onclick: move |_| {
                        let new_layout = match *layout.read() {
                            Layout::Split => Layout::OnlySidebar,
                            Layout::OnlyMain => Layout::Split,
                            Layout::OnlySidebar => Layout::Split,
                        };
                        layout.set(new_layout);
                    }
                }
                ActionbarButton {
                    text: "hi",
                    icon: &ld::LdPanelLeftOpen,
                    onclick: move |_| layout.set(Layout::OnlySidebar),
                }
                ActionbarButton {
                    text: "hi",
                    icon: &ld::LdPanelRightOpen,
                    onclick: move |_| layout.set(Layout::OnlyMain),
                }
                ActionbarButton {
                    text: "hi",
                    icon: &ld::LdPanelLeft,
                    onclick: move |_| layout.set(Layout::Split),
                }
            }

            div {
                class: "sbview-sidebar",
                div {
                    class: "sbview-sidebar-inner",
                    aside {
                        { sidebar }
                    }
                }
            }

            div {
                class: "sbview-divider",
                // save element reference for pointer capturing
                onpointerdown: move |evt| {
                    // prevent accidental text selection while dragging
                    evt.prevent_default();

                    // on web, we have access to web_sys and can properly
                    // capture the pointer. 
                    #[cfg(feature = "web")]
                    {
                        use web_sys::wasm_bindgen::JsCast;

                        let mut inner = move || -> anyhow::Result<()> {
                            let pointer_event = evt
                                .downcast::<web_sys::PointerEvent>()
                                .ok_or(anyhow!("cast failed"))?;
                        
                            let target = pointer_event
                                // must be current target because it seems like dioxus doesn't
                                // actually attach the event listener to the element we tell it 
                                // to. Rather it seems to do filtering after the fact.
                                .target()   
                                .ok_or(anyhow!("no target"))?
                                .dyn_into::<web_sys::Element>()
                                .map_err(|_| anyhow!("element cast failed"))?;
                            
                            target
                                .set_pointer_capture(pointer_event.pointer_id())
                                .map_err(|js_err| anyhow!("pointer capture failed: {js_err:?}"))?;
                            
                            dragging_pointer.set(Some(pointer_event.pointer_id()));
                            // calculate the offset to get new panel width from client coords
                            drag_offset.set((pointer_event.client_x() - (*sidebar_width.read() as i32)) as f64);
                        
                            Ok(())
                        };
                        
                        if let Err(err) = inner() {
                            tracing::error!("drag start failed: {:?}", err);
                        };
                    }
                    // Unfortunately we can't easily do pointer capture on
                    // non-web platforms, because we don't have access to the native
                    // web_sys data (rendering happens in native code).
                    #[cfg(not(feature = "web"))]
                    {
                        dragging_pointer.set(Some(evt.pointer_id()));
                        // calculate the offset to get new panel width from client coords
                        drag_offset.set(evt.client_coordinates().x - (*sidebar_width.read() as f64));
                    }
                },
                // these will be fired only when the pointer is captured
                // or moving above the divider in order to minimize
                // unnecessary event handler calls
                onpointermove: handle_resize_move,
                onpointerup: handle_resize_stop,
            }

            div {
                class: "sbview-main",
                div {
                    class: "sbview-main-inner",
                    main {
                        { main_view }
                    }
                }
            }
        }

    }
}