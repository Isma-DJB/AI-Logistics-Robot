# Security Policy

AI-Logistics-Robot is a simulation-first autonomous robotics project under
active development. Security and safety reports are taken seriously, even
though the project is not currently intended for production deployment.

## Supported Versions

Security corrections are applied only to the current state of the default
branch.

| Version | Supported |
| --- | --- |
| Current `main` branch | Yes |
| Historical implementation drafts and commits | No |
| Unmerged development branches | No |
| Third-party forks | No |

The repository does not currently publish stable production releases. A future
release policy will replace this table when versioned releases are introduced.

## Reporting a Vulnerability

Do not report suspected vulnerabilities through a public issue, pull request,
discussion, social-media post, or public comment.

Use GitHub private vulnerability reporting:

1. Open the repository `Security` page.
2. Select `Advisories`.
3. Select `Report a vulnerability`.
4. Submit the report privately to the repository maintainer.

Repository security page:

https://github.com/Isma-DJB/AI-Logistics-Robot/security

If private vulnerability reporting is temporarily unavailable, open a public
issue containing only a request for a private contact channel. Do not include
the vulnerability, reproduction steps, logs, credentials, or sensitive
information in that issue.

## Information to Include

A useful private report should contain:

- a concise description of the vulnerability;
- the affected version, branch, or commit;
- the affected component or interface;
- the potential security or safety impact;
- deterministic reproduction steps when possible;
- a minimal proof of concept when safe;
- sanitized logs, traces, or screenshots;
- known mitigations or workarounds;
- any planned public-disclosure date.

Remove credentials, access tokens, personal data, private keys, network
addresses, and unrelated confidential information before submission.

## Relevant Security and Safety Areas

Reports are especially useful when they concern:

- exposed credentials, tokens, secrets, or private configuration;
- dependency or software-supply-chain vulnerabilities;
- unauthorized command execution;
- bypass of validation, safety stops, or the manual safety rearm;
- inconsistent confirmed robot state after a failed operation;
- unsafe movement caused by malformed or untrusted input;
- denial of service or uncontrolled resource consumption;
- unsafe deserialization, file handling, or configuration parsing;
- future network, camera, microcontroller, or hardware interfaces;
- disclosure of personal or operationally sensitive information.

Safety defects that could cause unexpected physical movement should be reported
privately even when their exploitability is uncertain.

## Reports Outside This Policy

Use the public bug-report or feature-request forms for:

- ordinary functional defects without security or safety impact;
- documentation corrections;
- feature proposals;
- expected development limitations;
- unsupported historical versions;
- questions already answered by the project documentation.

The following reports may not receive a security response:

- automated scanner output without analysis or reproducible impact;
- purely theoretical concerns without an affected project component;
- vulnerabilities that exist only in unrelated third-party services;
- social-engineering attempts or requests for credentials;
- testing that requires disruption, data destruction, or unauthorized access.

## Coordinated Disclosure

Allow the maintainer reasonable time to investigate and prepare a correction
before publishing vulnerability details.

Do not exploit a vulnerability beyond the minimum necessary to demonstrate it.
Do not access other people's data, interrupt services, damage equipment, or
perform tests against systems without authorization.

The maintainer will:

- review reports in good faith;
- preserve confidentiality where reasonably possible;
- request additional information when needed;
- determine the affected scope and remediation plan;
- coordinate public disclosure when a correction is available.

Because this is currently a single-maintainer, non-production project, no fixed
response or resolution deadline is guaranteed.

## Accidental Secret Exposure

If a secret is committed or disclosed:

1. revoke or rotate it immediately;
2. remove it from current files;
3. inspect repository history and related systems;
4. report the incident privately;
5. do not publish the exposed value again.

Deleting a secret from the latest commit does not make the previous value safe.

## Good-Faith Research

Security research must comply with applicable laws and respect privacy,
ownership, and system availability.

Good-faith reports that follow this policy, avoid unnecessary harm, and provide
the maintainer with a reasonable opportunity to respond are welcomed.

## Language

Security reports may be submitted in English, German, or French. Repository
corrections and public technical documentation will remain in English.
