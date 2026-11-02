//! The customer quote: the shop's business details (kept between sessions), the customer and job, and a
//! printable page built from a priced job. The page is plain HTML with print styling, opened in the browser
//! to print or save as PDF.

use serde::{Deserialize, Serialize};

use super::logic::{Quote, Tool};

/// The shop's details, printed at the top of every quote.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Business {
    pub company: String,
    pub address: String,
    pub phone: String,
    pub email: String,
    pub website: String,
    /// Logo image file (PNG, JPEG, GIF or SVG).
    pub logo: String,
    /// Terms printed under the totals.
    pub terms: String,
}

/// Who the quote is for and which job.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Customer {
    pub name: String,
    pub contact: String,
    pub address: String,
    pub phone: String,
    pub email: String,
    pub job_name: String,
    pub job_number: String,
    pub po: String,
    pub valid_days: u32,
    pub notes: String,
}

impl Default for Customer {
    fn default() -> Self {
        Self {
            name: String::new(),
            contact: String::new(),
            address: String::new(),
            phone: String::new(),
            email: String::new(),
            job_name: String::new(),
            job_number: String::new(),
            po: String::new(),
            valid_days: 30,
            notes: String::new(),
        }
    }
}

/// What persists between sessions: the business and the shop's prices.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct Saved {
    pub business: Business,
    pub pricing: Option<SavedPricing>,
}

/// The pricing fields worth keeping (a copy so the file format doesn't follow every internal change).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct SavedPricing {
    pub steel_per_lb: f64,
    pub stainless_per_lb: f64,
    pub aluminum_per_lb: f64,
    pub machine_per_hour: f64,
    pub setup: f64,
    pub markup_pct: f64,
    pub scrap_allowance_pct: f64,
    pub per_pierce: f64,
    pub per_inch: f64,
    pub tax_pct: f64,
}

impl SavedPricing {
    pub fn from(p: &super::logic::Pricing) -> Self {
        Self {
            steel_per_lb: p.steel_per_lb,
            stainless_per_lb: p.stainless_per_lb,
            aluminum_per_lb: p.aluminum_per_lb,
            machine_per_hour: p.machine_per_hour,
            setup: p.setup,
            markup_pct: p.markup_pct,
            scrap_allowance_pct: p.scrap_allowance_pct,
            per_pierce: p.per_pierce,
            per_inch: p.per_inch,
            tax_pct: p.tax_pct,
        }
    }
    pub fn apply(&self, p: &mut super::logic::Pricing) {
        let ok = |v: f64| v.is_finite() && v >= 0.0;
        let set = |dst: &mut f64, v: f64| {
            if ok(v) {
                *dst = v;
            }
        };
        set(&mut p.steel_per_lb, self.steel_per_lb);
        set(&mut p.stainless_per_lb, self.stainless_per_lb);
        set(&mut p.aluminum_per_lb, self.aluminum_per_lb);
        set(&mut p.machine_per_hour, self.machine_per_hour);
        set(&mut p.setup, self.setup);
        set(&mut p.markup_pct, self.markup_pct);
        set(&mut p.scrap_allowance_pct, self.scrap_allowance_pct);
        set(&mut p.per_pierce, self.per_pierce);
        set(&mut p.per_inch, self.per_inch);
        set(&mut p.tax_pct, self.tax_pct);
    }
}

/// The settings file in SolidCraft's per-user data folder.
#[cfg(not(target_arch = "wasm32"))]
fn saved_path() -> Option<std::path::PathBuf> {
    Some(solidcraft_engine::recovery::data_dir()?.join("plasma_nest.json"))
}

#[cfg(not(target_arch = "wasm32"))]
pub fn load() -> Saved {
    saved_path().and_then(|p| std::fs::read(p).ok()).filter(|b| b.len() < 1 << 20).and_then(|b| serde_json::from_slice(&b).ok()).unwrap_or_default()
}

#[cfg(target_arch = "wasm32")]
pub fn load() -> Saved {
    Saved::default()
}

#[cfg(not(target_arch = "wasm32"))]
pub fn save(saved: &Saved) -> Result<(), String> {
    let path = saved_path().ok_or("no user data folder")?;
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    let text = serde_json::to_string_pretty(saved).map_err(|e| e.to_string())?;
    std::fs::write(path, text).map_err(|e| e.to_string())
}

#[cfg(target_arch = "wasm32")]
pub fn save(_: &Saved) -> Result<(), String> {
    Ok(())
}

/// Where printable quotes are written before opening.
#[cfg(not(target_arch = "wasm32"))]
pub fn quotes_dir() -> Option<std::path::PathBuf> {
    Some(solidcraft_engine::recovery::data_dir()?.join("quotes"))
}

/// Today's date (UTC) as year, month, day.
pub fn today() -> (i64, u32, u32) {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs()) as i64;
    civil(secs.div_euclid(86_400))
}

/// Days since 1970-01-01 to a calendar date (proleptic Gregorian).
pub fn civil(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

fn date_text((y, m, d): (i64, u32, u32)) -> String {
    const MONTHS: [&str; 12] =
        ["January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"];
    format!("{} {d}, {y}", MONTHS.get(m.saturating_sub(1) as usize).copied().unwrap_or(""))
}

fn esc(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _ => out.push(c),
        }
    }
    out
}

/// Escaped text with its line breaks kept.
fn lines(s: &str) -> String {
    s.trim().lines().map(|l| esc(l.trim())).collect::<Vec<_>>().join("<br>")
}

fn money(v: f64) -> String {
    let cents = (v * 100.0).round();
    if !cents.is_finite() {
        return "$0.00".into();
    }
    let neg = cents < 0.0;
    let cents = cents.abs() as u64;
    let dollars = (cents / 100).to_string();
    let mut grouped = String::new();
    for (i, ch) in dollars.chars().enumerate() {
        if i > 0 && (dollars.len() - i) % 3 == 0 {
            grouped.push(',');
        }
        grouped.push(ch);
    }
    format!("{}${grouped}.{:02}", if neg { "-" } else { "" }, cents % 100)
}

pub fn base64(bytes: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [chunk.first().copied().unwrap_or(0), chunk.get(1).copied().unwrap_or(0), chunk.get(2).copied().unwrap_or(0)];
        let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        for (i, shift) in [18, 12, 6, 0].iter().enumerate() {
            if i <= chunk.len() {
                out.push(char::from(T[((n >> shift) & 63) as usize]));
            } else {
                out.push('=');
            }
        }
    }
    out
}

/// A logo file as a data URI for the page, if it is an image type browsers show.
pub fn logo_uri(path: &str, bytes: &[u8]) -> Option<String> {
    let ext = path.rsplit('.').next()?.to_ascii_lowercase();
    let mime = match ext.as_str() {
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "gif" => "image/gif",
        "svg" => "image/svg+xml",
        "webp" => "image/webp",
        _ => return None,
    };
    (!bytes.is_empty() && bytes.len() < 8 << 20).then(|| format!("data:{mime};base64,{}", base64(bytes)))
}

/// One printable quote page.
pub fn html(b: &Business, c: &Customer, tool: &Tool, q: &Quote, tax_pct: f64, logo: Option<&str>, date: (i64, u32, u32)) -> String {
    use std::fmt::Write as _;
    let tax = q.total * tax_pct.max(0.0) / 100.0;
    let grand = q.total + tax;
    let valid = {
        let days = {
            // Days since epoch for `date`, then add the validity.
            let (y, m, d) = date;
            let y2 = if m <= 2 { y - 1 } else { y };
            let era = y2.div_euclid(400);
            let yoe = y2 - era * 400;
            let mp = if m > 2 { i64::from(m) - 3 } else { i64::from(m) + 9 };
            let doy = (153 * mp + 2) / 5 + i64::from(d) - 1;
            let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
            era * 146_097 + doe - 719_468
        };
        civil(days + i64::from(c.valid_days.min(3650)))
    };
    let material = match tool.thickness_in {
        Some(t) => format!("{} · {:.4}\" ({})", tool.metal.label(), t, tool.name),
        None => format!("{} ({})", tool.metal.label(), tool.name),
    };
    let quote_no = if c.job_number.trim().is_empty() { format!("{}{:02}{:02}", date.0, date.1, date.2) } else { c.job_number.trim().to_string() };
    let mut h = String::new();
    let _ = write!(
        h,
        r#"<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>Quote {qn}{company}</title>
<style>
  @page {{ size: letter; margin: 0.6in; }}
  * {{ box-sizing: border-box; }}
  body {{ font-family: "Segoe UI", system-ui, -apple-system, Helvetica, Arial, sans-serif; color: #1f2a37; margin: 0; background: #eef1f5; }}
  .page {{ max-width: 8.5in; margin: 24px auto; background: #fff; padding: 0.55in 0.6in; box-shadow: 0 2px 14px rgba(0,0,0,.12); }}
  header {{ display: flex; justify-content: space-between; align-items: flex-start; gap: 24px; border-bottom: 3px solid #236ead; padding-bottom: 18px; }}
  .brand {{ display: flex; gap: 16px; align-items: flex-start; }}
  .brand img {{ max-height: 84px; max-width: 200px; object-fit: contain; }}
  .company {{ font-size: 22px; font-weight: 700; margin: 0 0 4px; }}
  .muted {{ color: #5b6b7d; font-size: 13px; line-height: 1.45; }}
  .title {{ text-align: right; }}
  .title h1 {{ margin: 0; font-size: 30px; letter-spacing: 2px; color: #236ead; }}
  .title table td {{ padding: 1px 0 1px 14px; font-size: 13px; white-space: nowrap; }}
  .title table td:first-child {{ color: #5b6b7d; }}
  .cols {{ display: flex; gap: 32px; margin: 22px 0; }}
  .cols > div {{ flex: 1; }}
  h2 {{ font-size: 11px; letter-spacing: 1.2px; text-transform: uppercase; color: #5b6b7d; margin: 0 0 6px; }}
  .block {{ font-size: 14px; line-height: 1.5; }}
  table.items {{ width: 100%; border-collapse: collapse; margin-top: 8px; font-size: 14px; }}
  table.items th {{ text-align: left; background: #f1f5f9; color: #334155; font-weight: 600; padding: 9px 10px; border-bottom: 1px solid #cbd5e1; }}
  table.items td {{ padding: 9px 10px; border-bottom: 1px solid #e2e8f0; }}
  .num {{ text-align: right; white-space: nowrap; }}
  .totals {{ margin-left: auto; width: 300px; margin-top: 14px; font-size: 14px; }}
  .totals div {{ display: flex; justify-content: space-between; padding: 5px 10px; }}
  .totals .grand {{ background: #236ead; color: #fff; font-weight: 700; font-size: 17px; border-radius: 6px; margin-top: 6px; }}
  .notes {{ margin-top: 28px; font-size: 13px; line-height: 1.5; }}
  footer {{ margin-top: 40px; display: flex; justify-content: space-between; gap: 40px; font-size: 13px; }}
  .sign {{ flex: 1; border-top: 1px solid #94a3b8; padding-top: 6px; color: #5b6b7d; }}
  .bar {{ max-width: 8.5in; margin: 16px auto 0; text-align: right; }}
  .bar button {{ font: inherit; padding: 8px 18px; border: 0; border-radius: 6px; background: #236ead; color: #fff; cursor: pointer; }}
  @media print {{ body {{ background: #fff; }} .page {{ box-shadow: none; margin: 0; padding: 0; max-width: none; }} .bar {{ display: none; }} }}
</style></head><body>
<div class="bar"><button onclick="window.print()">Print or save as PDF</button></div>
<div class="page">
<header>
  <div class="brand">{logo}<div><p class="company">{company_h}</p><div class="muted">{address}{contact}</div></div></div>
  <div class="title"><h1>QUOTE</h1><table>
    <tr><td>Quote #</td><td>{qn}</td></tr>
    <tr><td>Date</td><td>{date}</td></tr>
    <tr><td>Valid until</td><td>{valid}</td></tr>{po}
  </table></div>
</header>
<div class="cols">
  <div><h2>Prepared for</h2><div class="block">{customer}</div></div>
  <div><h2>Job</h2><div class="block">{job}</div></div>
</div>
<table class="items"><thead><tr><th style="width:36px">#</th><th>Part</th><th class="num" style="width:70px">Qty</th><th class="num" style="width:110px">Unit price</th><th class="num" style="width:120px">Amount</th></tr></thead><tbody>
"#,
        qn = esc(&quote_no),
        company = if b.company.trim().is_empty() { String::new() } else { format!(" - {}", esc(b.company.trim())) },
        logo = logo.map(|u| format!(r#"<img src="{}" alt="">"#, esc(u))).unwrap_or_default(),
        company_h = if b.company.trim().is_empty() { "Your Company".to_string() } else { esc(b.company.trim()) },
        address = lines(&b.address),
        contact = [b.phone.trim(), b.email.trim(), b.website.trim()]
            .iter()
            .filter(|s| !s.is_empty())
            .map(|s| format!("<br>{}", esc(s)))
            .collect::<String>(),
        date = date_text(date),
        valid = date_text(valid),
        po = if c.po.trim().is_empty() { String::new() } else { format!("\n    <tr><td>Customer PO</td><td>{}</td></tr>", esc(c.po.trim())) },
        customer = {
            let mut v = Vec::new();
            if !c.name.trim().is_empty() {
                v.push(format!("<strong>{}</strong>", esc(c.name.trim())));
            }
            if !c.contact.trim().is_empty() {
                v.push(format!("Attn: {}", esc(c.contact.trim())));
            }
            if !c.address.trim().is_empty() {
                v.push(lines(&c.address));
            }
            for s in [c.phone.trim(), c.email.trim()] {
                if !s.is_empty() {
                    v.push(esc(s));
                }
            }
            if v.is_empty() { "&nbsp;".to_string() } else { v.join("<br>") }
        },
        job = {
            let mut v = Vec::new();
            if !c.job_name.trim().is_empty() {
                v.push(format!("<strong>{}</strong>", esc(c.job_name.trim())));
            }
            if !c.job_number.trim().is_empty() {
                v.push(format!("Job # {}", esc(c.job_number.trim())));
            }
            v.push(format!("Material: {}", esc(&material)));
            v.push(format!("{} parts · CNC plasma cut", q.lines.iter().map(|l| u64::from(l.qty)).sum::<u64>()));
            v.join("<br>")
        },
    );
    for (i, l) in q.lines.iter().filter(|l| l.qty > 0).enumerate() {
        let name = l.name.trim_end_matches(".dxf").trim_end_matches(".DXF");
        let _ = writeln!(
            h,
            r#"<tr><td>{}</td><td>{}</td><td class="num">{}</td><td class="num">{}</td><td class="num">{}</td></tr>"#,
            i + 1,
            esc(name),
            l.qty,
            money(l.each),
            money(l.total)
        );
    }
    let _ = write!(
        h,
        r#"</tbody></table>
<div class="totals">
  <div><span>Subtotal</span><span>{}</span></div>"#,
        money(q.total)
    );
    if tax_pct > 0.0 {
        let _ = write!(
            h,
            r#"
  <div><span>Tax ({}%)</span><span>{}</span></div>"#,
            trim_num(tax_pct),
            money(tax)
        );
    }
    let _ = write!(
        h,
        r#"
  <div class="grand"><span>Total</span><span>{}</span></div>
</div>"#,
        money(grand)
    );
    let mut notes = Vec::new();
    if !c.notes.trim().is_empty() {
        notes.push(format!("<h2>Notes</h2><div>{}</div>", lines(&c.notes)));
    }
    if !b.terms.trim().is_empty() {
        notes.push(format!("<h2 style=\"margin-top:14px\">Terms</h2><div>{}</div>", lines(&b.terms)));
    }
    if !notes.is_empty() {
        let _ = write!(h, "\n<div class=\"notes\">{}</div>", notes.join(""));
    }
    let _ = write!(
        h,
        r#"
<footer><div class="sign">Accepted by (signature)</div><div class="sign">Date</div></footer>
</div></body></html>
"#
    );
    h
}

fn trim_num(v: f64) -> String {
    let s = format!("{v:.3}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dates_base64_and_money() {
        assert_eq!(civil(0), (1970, 1, 1));
        assert_eq!(civil(20_758), (2026, 11, 1));
        assert_eq!(civil(11_016), (2000, 2, 29));
        assert_eq!(base64(b"Man"), "TWFu");
        assert_eq!(base64(b"Ma"), "TWE=");
        assert_eq!(base64(b"M"), "TQ==");
        assert_eq!(base64(b""), "");
        assert_eq!(money(1234567.891), "$1,234,567.89");
        assert_eq!(money(0.5), "$0.50");
        assert_eq!(money(f64::NAN), "$0.00");
        assert!(logo_uri("logo.PNG", b"x").unwrap().starts_with("data:image/png;base64,"));
        assert!(logo_uri("logo.exe", b"x").is_none());
    }

    #[test]
    fn the_quote_page_carries_the_business_customer_lines_and_totals() {
        let tools = super::super::logic::default_tools();
        let tool = tools.iter().find(|t| t.number == 61).unwrap();
        let parts = super::super::logic::sample_parts();
        let q = super::super::logic::quote(&parts, tool, &super::super::logic::Pricing::default(), None);
        let b = Business {
            company: "Acme <Plasma> & Fab".into(),
            address: "12 Main St\nSpringfield, IL 62701".into(),
            phone: "(555) 010-0199".into(),
            terms: "Net 30".into(),
            ..Business::default()
        };
        let c = Customer { name: "Bob's Trailers".into(), job_number: "Q-1042".into(), job_name: "Hitch plates".into(), ..Customer::default() };
        let page = html(&b, &c, tool, &q, 7.5, Some("data:image/png;base64,AAAA"), (2026, 11, 1));
        assert!(page.contains("Acme &lt;Plasma&gt; &amp; Fab"));
        assert!(page.contains("12 Main St<br>Springfield, IL 62701"));
        assert!(page.contains("Bob&#39;s Trailers") && page.contains("Q-1042") && page.contains("Hitch plates"));
        assert!(page.contains("November 1, 2026") && page.contains("December 1, 2026"));
        assert!(page.contains("base_plate") && page.contains("Tax (7.5%)") && page.contains("Net 30"));
        assert!(page.contains(&money(q.total * 1.075)));
        assert!(page.contains("<img src=\"data:image/png;base64,AAAA\""));
        // Nothing the user typed can open a tag.
        let evil = Customer { notes: "<script>alert(1)</script>".into(), ..Customer::default() };
        assert!(!html(&b, &evil, tool, &q, 0.0, None, (2026, 1, 31)).contains("<script>"));
    }
}
