# storage

Custom inventory management system.

## Motivation

This project started out as the "physical" component of my inventory system based on [InvenTree](https://inventree.org), with the goal of tightly integrating physical devices (scanners, scales, ...) into my InvenTree workflow.

However I also wanted to change quite a few little things about InvenTree itself and instead of fighting to do so in a large codebase I am not familiar with, I am now experimenting with creating my own system that does only what I need but exactly the way I want it.

That is not to say that InvenTree is bad software that's hard to customize. Creating my own system from scratch simply gives me more freedom to experiment. If you are looking for an inventory management solution for yourself I still recommend using InvenTree as it is much more stable and flexible for most use cases.


## Repository structure

- `packages`: The core storage application crates.
  - `app`: Main crate, serving as the Dioxus entrypoint.
  - `frontend`: Any crates containing frontend code (UI, ...). Note that these are also compiled on the server for the purpose of SSR.
  - `backend`: Any crates exclusively used on the backend. These are only compiled in the backend.
    - `storage_core`: Core application logic.
    - `migration`: SeaORM DB migrations
    - `entity`: SeaORM generated entity definitions.
  - `api`: Server functions that bridge frontend and backend
- `devices`: Hardware and firmware for devices that directly integrate with the storage application.
- `scripts`: Development utilities


## Architecture

The main application is essentially a Dioxus Fullstack application with a heavier server core bolted on. 

[![Dependency diagram](<images/shapes at 26-07-31 17.12.37.png>)](https://www.tldraw.com/f/r18U9CWPqxqCzgZosVHZ_)

The Dioxus entrypoint is the [app crate](packages/app/), which behaves like any dioxus fullstack application, depending on the enabled features.

In client mode, it simply launches a dioxus client application, passing it the main component imported from a frontend crate (for now, we use [web](packages/frontend/web/) for all targets, as this project is primarily intended to be a webapp, or PWA).

In server mode, it initiates all the backend application components imported from the [storage_core crate](packages/backend/storage_core/) and registers server functions plus the main component imported from the frontend with an axum router.

The API crate contains server functions and only depends on `storage_core` with the server feature. It is the only way for the frontend to communicate with the backend. This way the `storage_core` crate (and all it's backend-only dependencies) does not need to worry about the separation between backend and frontend, as it is only ever compiled for the backend.

This clear split was chosen because the storage server is more than just the backend of the GUI app, also interfacing with various other components (e.g. custom hardware devices). The GUI is just one (admittedly large and important) component of the greater system.
