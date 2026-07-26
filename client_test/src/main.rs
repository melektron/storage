use dioxus::prelude::*;

const FAVICON: Asset = asset!("/assets/favicon.ico");
const MAIN_CSS: Asset = asset!("/assets/main.css");
const HEADER_SVG: Asset = asset!("/assets/header.svg");
const TAILWIND_CSS: Asset = asset!("/assets/tailwind.css");

fn main() {
    dioxus::launch(App);
}

#[component]
fn App() -> Element {
    rsx! {
        document::Link { rel: "icon", href: FAVICON }
        document::Link { rel: "stylesheet", href: MAIN_CSS } 
        document::Link { rel: "stylesheet", href: TAILWIND_CSS }
        Hero {}

    }
}

#[component]
pub fn Hero() -> Element {
    let ab = "Hello";
    rsx! {
        h1 { "hi" }
        div {
            id: "hero",
            img { src: HEADER_SVG, id: "header" }
            div { id: "links",
                LinkButton { target: "https://dioxuslabs.com/learn/0.7/", text: "📚 Learn Dioxus {ab}" }
                a { href: "https://dioxuslabs.com/awesome", "🚀 Awesome Dioxus" }
                a { href: "https://github.com/dioxus-community/", "📡 Community Libraries" }
                a { href: "https://github.com/DioxusLabs/sdk", "⚙️ Dioxus Development Kit" }
                a { href: "https://marketplace.visualstudio.com/items?itemName=DioxusLabs.dioxus", "💫 VSCode Extension" }
                a { href: "https://discord.gg/XgGxMSkvUM", "👋 Community Discord" }
            }
            ul {
                for number in 1..5 {
                    li { 
                        "{number}"
                    }
                }
            }
        }
    }
}

/// This displays a special link that looks like a button
#[component]
fn LinkButton(
    /// The target URL the link should point to
    target: String,

    /// The text to display
    text: String
) -> Element {
    rsx! {
        a {
            class: "link_button",
            class: "bg-red-300",
            href: target,
            {text}
        }
    }
}