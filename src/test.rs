#[cfg(test)]
mod tests {
    #[test]
    fn splitting() {
        let v = &[0, 1, 2, 3, 4, 5];
        if let Some(index) = v.iter().position(|&r| r == 2) {
            let end = if let Some(i) = v[index..].iter().position(|&r| r == 7) {
                i + index + 1
            } else {
                v.len()
            };
            let v1: Vec<_> = v[index..end].iter().clone().collect();

            println!("{:?}, ({} -> {})", v1, index, end);
        }
    }
}
