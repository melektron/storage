fn main() {
    let a = "Hello";

    println!("Hello, world! {}", test(a));
}

fn test(a: &str) -> &str {
    a
}
