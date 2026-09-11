//! The comparison a hardware plan makes against the registers carrying it.
//!
//! Both plans check a read-back the same way. A field has to carry the value
//! the plan asked for, a flag has to stand one way, a counter has to name a
//! position inside its buffer, and the first requirement that does not hold is
//! the fault the caller gets. That rule lives here once.
//!
//! What each plan declares is its own list of fields and its own fault names.
//! The output path emits, so it generates the clocks and can underfeed. The
//! input path follows another interface and can overfeed. They refuse over
//! different fields, under separate fault types, through one reading.

/// Refuses on the first requirement of `$seen` that does not hold.
///
/// Takes the read-back to read, the fault type to build, then one clause per
/// requirement. A clause names the fault first, then the field and what has to
/// be true of it:
///
/// - `carries` holds when the field equals the value,
/// - `rejects` holds when it does not,
/// - `set` and `clear` hold on a flag standing that way,
/// - `at most` holds when the field is no greater than the bound.
///
/// Clauses expand in the order written and each returns, so a read-back with
/// several bad fields produces the fault of the first clause listed. Every
/// clause ends with a comma, the last one included.
macro_rules! refuse_unless
{
    ($seen:ident, $fault:ident $(,)?) =>
    {
    };

    (
        $seen:ident, $fault:ident,
        $variant:ident unless $field:ident carries $want:expr,
        $($rest:tt)*
    ) =>
    {
        if $seen.$field != $want
        {
            return Err($fault::$variant);
        }

        refuse_unless!($seen, $fault, $($rest)*);
    };

    (
        $seen:ident, $fault:ident,
        $variant:ident unless $field:ident rejects $refused:expr,
        $($rest:tt)*
    ) =>
    {
        if $seen.$field == $refused
        {
            return Err($fault::$variant);
        }

        refuse_unless!($seen, $fault, $($rest)*);
    };

    (
        $seen:ident, $fault:ident,
        $variant:ident unless $field:ident at most $bound:expr,
        $($rest:tt)*
    ) =>
    {
        if $seen.$field > $bound
        {
            return Err($fault::$variant);
        }

        refuse_unless!($seen, $fault, $($rest)*);
    };

    (
        $seen:ident, $fault:ident,
        $variant:ident unless $field:ident set,
        $($rest:tt)*
    ) =>
    {
        if !$seen.$field
        {
            return Err($fault::$variant);
        }

        refuse_unless!($seen, $fault, $($rest)*);
    };

    (
        $seen:ident, $fault:ident,
        $variant:ident unless $field:ident clear,
        $($rest:tt)*
    ) =>
    {
        if $seen.$field
        {
            return Err($fault::$variant);
        }

        refuse_unless!($seen, $fault, $($rest)*);
    };
}

pub(crate) use refuse_unless;

/// Reads the cause byte of every variant of one fault enum against the number
/// written here.
///
/// The discriminant is what a fault record carries and what a probe looks up,
/// so a variant that changes number changes the meaning of a parked board. The
/// numbers are therefore stated a second time, away from the declaration, and
/// the cast is what says which one the type really carries.
///
/// The same clause list builds the array the loop walks and the arms of the
/// match inside it, so no variant can be read and left unpinned. A variant
/// added to the enum and not added here leaves that match without an arm for
/// it, which does not compile, and the arm cannot be written anywhere but in
/// the clause list that also feeds the array.
#[cfg(test)]
macro_rules! pin_cause_bytes
{
    ($fault:ident { $($variant:ident = $code:literal,)+ }) =>
    {
        for fault in [$($fault::$variant,)+]
        {
            let pinned: u8 = match fault
            {
                $($fault::$variant => $code,)+
            };

            assert_eq!
            (
                fault as u8,
                pinned,
                "{fault:?} carries a cause byte the fault record encoding does not pin"
            );
            assert_ne!(fault as u8, 0, "{fault:?} numbers a cause zero");
        }
    };
}

#[cfg(test)]
pub(crate) use pin_cause_bytes;
