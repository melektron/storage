use dioxus::document::eval;
use dioxus::prelude::*;

#[component]
pub fn Camera() -> Element {
    let mut running = use_signal(|| false);
    let mut facing = use_signal(|| "environment");
    let mut captured_image = use_signal(|| Option::<String>::None);
    let mut error = use_signal(|| Option::<String>::None);

    // Start the camera.
    let start_camera = move |_| {
        let facing = facing();

        spawn(async move {
            let js = format!(
                r#"
                (async () => {{
                    const video = document.getElementById("camera-video");

                    if (!video) {{
                        throw new Error("Camera video element not found");
                    }}

                    // Stop any existing stream first.
                    if (window.__cameraStream) {{
                        window.__cameraStream
                            .getTracks()
                            .forEach(track => track.stop());

                        window.__cameraStream = null;
                    }}

                    try {{
                        const stream = await navigator.mediaDevices.getUserMedia({{
                            video: {{
                                facingMode: {{
                                    ideal: "{facing}"
                                }}
                            }},
                            audio: false
                        }});

                        window.__cameraStream = stream;
                        video.srcObject = stream;

                        await video.play();

                        return "ok";
                    }} catch (e) {{
                        throw new Error(e.message || String(e));
                    }}
                }})()
                "#
            );

            match eval(&js).await {
                Ok(_) => {
                    running.set(true);
                    error.set(None);
                }
                Err(e) => {
                    running.set(false);
                    error.set(Some(e.to_string()));
                }
            }
        });
    };

    // Stop the camera.
    let stop_camera = move |_| {
        spawn(async move {
            let _ = eval(
                r#"
                (() => {
                    if (window.__cameraStream) {
                        window.__cameraStream
                            .getTracks()
                            .forEach(track => track.stop());

                        window.__cameraStream = null;
                    }

                    const video = document.getElementById("camera-video");

                    if (video) {
                        video.srcObject = null;
                    }

                    return "ok";
                })()
                "#,
            )
            .await;

            running.set(false);
        });
    };

    // Switch between front and rear camera.
    let switch_camera = move |_| {
        let new_facing = if facing() == "environment" {
            "user"
        } else {
            "environment"
        };

        facing.set(new_facing);

        // Restart the stream using the new facing mode.
        spawn(async move {
            let js = format!(
                r#"
                (async () => {{
                    const video = document.getElementById("camera-video");

                    if (window.__cameraStream) {{
                        window.__cameraStream
                            .getTracks()
                            .forEach(track => track.stop());
                    }}

                    const stream = await navigator.mediaDevices.getUserMedia({{
                        video: {{
                            facingMode: {{
                                ideal: "{new_facing}"
                            }}
                        }},
                        audio: false
                    }});

                    window.__cameraStream = stream;
                    video.srcObject = stream;

                    await video.play();

                    return "ok";
                }})()
                "#
            );

            match eval(&js).await {
                Ok(_) => {
                    running.set(true);
                    error.set(None);
                }
                Err(e) => {
                    error.set(Some(e.to_string()));
                }
            }
        });
    };

    // Capture a frame from the video.
    let capture = move |_| {
        spawn(async move {
            let result = eval(
                r#"
                (() => {
                    const video = document.getElementById("camera-video");

                    if (!video || !video.srcObject) {
                        throw new Error("Camera is not running");
                    }

                    if (video.readyState < 2) {
                        throw new Error("Camera video is not ready");
                    }

                    const canvas = document.createElement("canvas");

                    canvas.width = video.videoWidth;
                    canvas.height = video.videoHeight;

                    const ctx = canvas.getContext("2d");

                    if (!ctx) {
                        throw new Error("Could not create canvas context");
                    }

                    ctx.drawImage(
                        video,
                        0,
                        0,
                        canvas.width,
                        canvas.height
                    );

                    return canvas.toDataURL("image/jpeg", 0.9);
                })()
                "#,
            )
            .await;

            match result {
                Ok(image) => {
                    captured_image.set(Some(image.to_string()));
                    error.set(None);
                }
                Err(e) => {
                    error.set(Some(e.to_string()));
                }
            }
        });
    };

    // Clean up the camera when the component is destroyed.
    use_drop(move || {
        // This is fire-and-forget because the component is being destroyed.
        spawn(async move {
            let _ = eval(
                r#"
                (() => {
                    if (window.__cameraStream) {
                        window.__cameraStream
                            .getTracks()
                            .forEach(track => track.stop());

                        window.__cameraStream = null;
                    }
                })()
                "#,
            )
            .await;
        });
    });

    rsx! {
        div {
            class: "camera",

            div {
                class: "camera-preview",

                video {
                    id: "camera-video",
                    autoplay: true,
                    playsinline: true,
                    muted: true,
                }
            }

            div {
                class: "camera-controls",

                button {
                    onclick: start_camera,
                    disabled: running(),
                    "Start"
                }

                button {
                    onclick: stop_camera,
                    disabled: !running(),
                    "Stop"
                }

                button {
                    onclick: switch_camera,
                    disabled: !running(),
                    "Switch camera"
                }

                button {
                    onclick: capture,
                    disabled: !running(),
                    "Capture"
                }
            }

            if let Some(err) = error() {
                p {
                    class: "camera-error",
                    "{err}"
                }
            }

            if let Some(image) = captured_image() {
                div {
                    class: "captured",

                    h3 {
                        "Captured image"
                    }

                    img {
                        src: "{image}",
                    }
                }
            }
        }
    }
}