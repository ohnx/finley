//! Minimal JSON reader. Only what the map/scenario schema needs.

use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(BTreeMap<String, Json>),
}

impl Json {
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(m) => m.get(key),
            _ => None,
        }
    }

    pub fn arr(&self) -> Option<&Vec<Json>> {
        match self {
            Json::Arr(a) => Some(a),
            _ => None,
        }
    }

    pub fn str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    pub fn num(&self) -> Option<f64> {
        match self {
            Json::Num(n) => Some(*n),
            _ => None,
        }
    }

    pub fn usize_at(&self, key: &str) -> Option<usize> {
        self.get(key).and_then(|v| v.num()).map(|n| n as usize)
    }

    pub fn u32_at(&self, key: &str) -> Option<u32> {
        self.get(key).and_then(|v| v.num()).map(|n| n as u32)
    }

    pub fn f32_at(&self, key: &str) -> Option<f32> {
        self.get(key).and_then(|v| v.num()).map(|n| n as f32)
    }

    pub fn str_at(&self, key: &str) -> Option<String> {
        self.get(key).and_then(|v| v.str()).map(|s| s.to_string())
    }

    /// Read a `[x, y]` pair.
    pub fn pair(&self) -> Option<(usize, usize)> {
        let a = self.arr()?;
        if a.len() != 2 {
            return None;
        }
        Some((a[0].num()? as usize, a[1].num()? as usize))
    }
}

pub fn parse(src: &str) -> Result<Json, String> {
    let b: Vec<char> = src.chars().collect();
    let mut i = 0usize;
    let v = parse_value(&b, &mut i)?;
    skip_ws(&b, &mut i);
    if i != b.len() {
        return Err(format!("trailing input at char {}", i));
    }
    Ok(v)
}

fn skip_ws(b: &[char], i: &mut usize) {
    while *i < b.len() && (b[*i] == ' ' || b[*i] == '\n' || b[*i] == '\t' || b[*i] == '\r') {
        *i += 1;
    }
}

fn parse_value(b: &[char], i: &mut usize) -> Result<Json, String> {
    skip_ws(b, i);
    if *i >= b.len() {
        return Err("unexpected end of input".to_string());
    }
    match b[*i] {
        '{' => parse_obj(b, i),
        '[' => parse_arr(b, i),
        '"' => Ok(Json::Str(parse_str(b, i)?)),
        't' => {
            expect(b, i, "true")?;
            Ok(Json::Bool(true))
        }
        'f' => {
            expect(b, i, "false")?;
            Ok(Json::Bool(false))
        }
        'n' => {
            expect(b, i, "null")?;
            Ok(Json::Null)
        }
        _ => parse_num(b, i),
    }
}

fn expect(b: &[char], i: &mut usize, word: &str) -> Result<(), String> {
    for c in word.chars() {
        if *i >= b.len() || b[*i] != c {
            return Err(format!("expected `{}` at char {}", word, i));
        }
        *i += 1;
    }
    Ok(())
}

fn parse_obj(b: &[char], i: &mut usize) -> Result<Json, String> {
    *i += 1; // consume '{'
    let mut m = BTreeMap::new();
    skip_ws(b, i);
    if *i < b.len() && b[*i] == '}' {
        *i += 1;
        return Ok(Json::Obj(m));
    }
    loop {
        skip_ws(b, i);
        let k = parse_str(b, i)?;
        skip_ws(b, i);
        if *i >= b.len() || b[*i] != ':' {
            return Err(format!("expected `:` at char {}", i));
        }
        *i += 1;
        let v = parse_value(b, i)?;
        m.insert(k, v);
        skip_ws(b, i);
        if *i >= b.len() {
            return Err("unterminated object".to_string());
        }
        match b[*i] {
            ',' => {
                *i += 1;
            }
            '}' => {
                *i += 1;
                return Ok(Json::Obj(m));
            }
            _ => return Err(format!("expected `,` or `}}` at char {}", i)),
        }
    }
}

fn parse_arr(b: &[char], i: &mut usize) -> Result<Json, String> {
    *i += 1; // consume '['
    let mut a = Vec::new();
    skip_ws(b, i);
    if *i < b.len() && b[*i] == ']' {
        *i += 1;
        return Ok(Json::Arr(a));
    }
    loop {
        let v = parse_value(b, i)?;
        a.push(v);
        skip_ws(b, i);
        if *i >= b.len() {
            return Err("unterminated array".to_string());
        }
        match b[*i] {
            ',' => {
                *i += 1;
            }
            ']' => {
                *i += 1;
                return Ok(Json::Arr(a));
            }
            _ => return Err(format!("expected `,` or `]` at char {}", i)),
        }
    }
}

fn parse_str(b: &[char], i: &mut usize) -> Result<String, String> {
    if *i >= b.len() || b[*i] != '"' {
        return Err(format!("expected string at char {}", i));
    }
    *i += 1;
    let mut s = String::new();
    while *i < b.len() {
        let c = b[*i];
        *i += 1;
        match c {
            '"' => return Ok(s),
            '\\' => {
                if *i >= b.len() {
                    return Err("bad escape".to_string());
                }
                let e = b[*i];
                *i += 1;
                match e {
                    'n' => s.push('\n'),
                    't' => s.push('\t'),
                    'r' => s.push('\r'),
                    '"' => s.push('"'),
                    '\\' => s.push('\\'),
                    '/' => s.push('/'),
                    _ => return Err(format!("unsupported escape `{}`", e)),
                }
            }
            _ => s.push(c),
        }
    }
    Err("unterminated string".to_string())
}

fn parse_num(b: &[char], i: &mut usize) -> Result<Json, String> {
    let start = *i;
    if *i < b.len() && (b[*i] == '-' || b[*i] == '+') {
        *i += 1;
    }
    while *i < b.len() && (b[*i].is_ascii_digit() || b[*i] == '.' || b[*i] == 'e' || b[*i] == 'E' || b[*i] == '-' || b[*i] == '+') {
        *i += 1;
    }
    let s: String = b[start..*i].iter().collect();
    s.parse::<f64>()
        .map(Json::Num)
        .map_err(|_| format!("bad number `{}` at char {}", s, start))
}
